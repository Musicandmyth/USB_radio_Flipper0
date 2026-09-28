/**
 * USB Radio — use a USB SDR (RTL-SDR, HackRF, ...) as a radio module for
 * the Flipper Zero.
 *
 * The Flipper cannot be a USB host, so the SDR is driven by a small bridge
 * program on the PC (host/usbradio_bridge.py). This app takes over the
 * Flipper's USB virtual COM port and speaks a compact framed protocol with
 * the bridge: the Flipper is the tuner/UI, the PC does the DSP.
 *
 * Features:
 *  - live spectrum analyzer (128 bins) with tunable center frequency,
 *    step and span
 *  - OOK/ASK signal capture: the bridge demodulates pulses and the app
 *    writes a standard Sub-GHz RAW .sub file to the SD card, replayable
 *    with the Flipper's own CC1101 radio.
 */
#include <furi.h>
#include <furi_hal_usb.h>
#include <furi_hal_usb_cdc.h>
#include <furi_hal_rtc.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>
#include <cli/cli_vcp.h>

#include "usb_radio_proto.h"

#define TAG "UsbRadio"

#define CDC_IF_NUM 0
#define LINK_TIMEOUT_MS 3000
#define PING_PERIOD_MS 500
#define NOTICE_MS 3000
#define SPECTRUM_FPS 15

#define CAPTURE_DIR EXT_PATH("subghz")
#define CAPTURE_VALUES_PER_LINE 256

#define FREQ_MIN 1000000UL
#define FREQ_MAX 4290000000UL

typedef enum {
    UrStateWaitHost,
    UrStateSpectrum,
    UrStateCapture,
} UrState;

typedef enum {
    WorkerEvtStop = (1 << 0),
    WorkerEvtCdcRx = (1 << 1),
} WorkerEvt;

static const uint32_t step_table[] =
    {1000, 5000, 10000, 25000, 50000, 100000, 1000000, 10000000};
static const char* const step_labels[] = {"1k", "5k", "10k", "25k", "50k", "100k", "1M", "10M"};

static const uint32_t span_table[] =
    {250000, 1000000, 2000000, 2400000, 3200000, 8000000, 10000000};
static const char* const span_labels[] = {"250k", "1M", "2M", "2.4M", "3.2M", "8M", "10M"};

typedef struct {
    Gui* gui;
    ViewPort* view_port;
    FuriMessageQueue* input_queue;
    FuriMutex* mutex;

    FuriHalUsbInterface* usb_prev;
    CliVcp* cli_vcp;
    FuriThread* worker;
    FuriSemaphore* tx_done;

    bool running;
    bool dtr; /* host has the COM port open */
    bool connected; /* bridge HELLO received and link alive */
    uint32_t last_rx_tick;
    char host_name[23];

    UrState state;
    uint32_t freq_hz;
    uint8_t step_idx;
    uint8_t span_idx;

    uint8_t bins[UR_SPECTRUM_BINS];
    bool have_spectrum;

    Storage* storage;
    File* cap_file;
    FuriString* cap_line;
    uint32_t cap_line_values;
    uint32_t cap_pulses;
    uint32_t cap_start_tick;
    char cap_name[40];

    char notice[48];
    uint32_t notice_until;
} UsbRadioApp;

/* ------------------------------------------------------------------ */
/* Link: frame TX                                                      */
/* ------------------------------------------------------------------ */

static void ur_send_frame(UsbRadioApp* app, uint8_t type, const void* payload, uint8_t len) {
    furi_assert(len <= 58);
    uint8_t buf[64];
    buf[0] = UR_MAGIC1;
    buf[1] = UR_MAGIC2;
    buf[2] = type;
    buf[3] = len;
    if(len) memcpy(&buf[4], payload, len);
    uint8_t chk = type ^ len;
    for(uint8_t i = 0; i < len; i++) chk ^= buf[4 + i];
    buf[4 + len] = chk;

    /* Wait for the previous packet to leave the endpoint. If no host is
     * reading, the semaphore may never release (the packet sits in the
     * endpoint); time out and send anyway so the link stays alive once a
     * host does show up — a lost frame is fine, a wedged link is not. */
    furi_semaphore_acquire(app->tx_done, 100);
    furi_hal_cdc_send(CDC_IF_NUM, buf, len + 5);
}

static void ur_send_config(UsbRadioApp* app) {
    UrConfig cfg = {
        .mode = (app->state == UrStateSpectrum) ? UR_MODE_SPECTRUM :
                (app->state == UrStateCapture)  ? UR_MODE_CAPTURE :
                                                  UR_MODE_IDLE,
        .freq_hz = app->freq_hz,
        .span_hz = span_table[app->span_idx],
        .gain_db_tenths = UR_GAIN_AUTO,
        .fps = SPECTRUM_FPS,
    };
    ur_send_frame(app, UR_MSG_CONFIG, &cfg, sizeof(cfg));
}

/* ------------------------------------------------------------------ */
/* Capture file (.sub RAW)                                             */
/* ------------------------------------------------------------------ */

static void ur_notice(UsbRadioApp* app, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(app->notice, sizeof(app->notice), fmt, args);
    va_end(args);
    app->notice_until = furi_get_tick() + furi_ms_to_ticks(NOTICE_MS);
}

/* Called with app->mutex held. */
static bool ur_capture_start(UsbRadioApp* app) {
    storage_simply_mkdir(app->storage, CAPTURE_DIR);

    DateTime dt;
    furi_hal_rtc_get_datetime(&dt);
    snprintf(
        app->cap_name,
        sizeof(app->cap_name),
        "usbr-%02u%02u%02u-%02u%02u%02u.sub",
        dt.year % 100,
        dt.month,
        dt.day,
        dt.hour,
        dt.minute,
        dt.second);

    FuriString* path = furi_string_alloc_printf("%s/%s", CAPTURE_DIR, app->cap_name);
    app->cap_file = storage_file_alloc(app->storage);
    bool ok = storage_file_open(
        app->cap_file, furi_string_get_cstr(path), FSAM_WRITE, FSOM_CREATE_ALWAYS);
    furi_string_free(path);

    if(!ok) {
        storage_file_free(app->cap_file);
        app->cap_file = NULL;
        ur_notice(app, "SD write failed");
        return false;
    }

    FuriString* header = furi_string_alloc_printf(
        "Filetype: Flipper SubGhz RAW File\n"
        "Version: 1\n"
        "Frequency: %lu\n"
        "Preset: FuriHalSubGhzPresetOok650Async\n"
        "Protocol: RAW\n",
        app->freq_hz);
    storage_file_write(app->cap_file, furi_string_get_cstr(header), furi_string_size(header));
    furi_string_free(header);

    furi_string_set(app->cap_line, "RAW_Data: ");
    app->cap_line_values = 0;
    app->cap_pulses = 0;
    app->cap_start_tick = furi_get_tick();
    return true;
}

/* Called with app->mutex held. */
static void ur_capture_append(UsbRadioApp* app, const int32_t* durations, uint8_t count) {
    if(!app->cap_file) return;
    for(uint8_t i = 0; i < count; i++) {
        furi_string_cat_printf(app->cap_line, "%ld ", (long)durations[i]);
        app->cap_line_values++;
        app->cap_pulses++;
        if(app->cap_line_values >= CAPTURE_VALUES_PER_LINE) {
            furi_string_cat(app->cap_line, "\n");
            storage_file_write(
                app->cap_file,
                furi_string_get_cstr(app->cap_line),
                furi_string_size(app->cap_line));
            furi_string_set(app->cap_line, "RAW_Data: ");
            app->cap_line_values = 0;
        }
    }
}

/* Called with app->mutex held. */
static void ur_capture_stop(UsbRadioApp* app) {
    if(!app->cap_file) return;
    if(app->cap_line_values) {
        furi_string_cat(app->cap_line, "\n");
        storage_file_write(
            app->cap_file, furi_string_get_cstr(app->cap_line), furi_string_size(app->cap_line));
    }
    storage_file_close(app->cap_file);
    storage_file_free(app->cap_file);
    app->cap_file = NULL;
    if(app->cap_pulses) {
        ur_notice(app, "Saved %s", app->cap_name);
    } else {
        ur_notice(app, "No pulses captured");
    }
}

/* ------------------------------------------------------------------ */
/* Link: frame RX (worker thread)                                      */
/* ------------------------------------------------------------------ */

static void ur_handle_frame(UsbRadioApp* app, uint8_t type, const uint8_t* payload, uint8_t len) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    app->last_rx_tick = furi_get_tick();

    switch(type) {
    case UR_MSG_HELLO:
        if(len >= sizeof(UrHello)) {
            const UrHello* hello = (const UrHello*)payload;
            memcpy(app->host_name, hello->name, sizeof(hello->name));
            app->host_name[sizeof(app->host_name) - 1] = '\0';
            if(!app->connected) {
                app->connected = true;
                FURI_LOG_I(TAG, "Bridge connected: %s", app->host_name);
                if(app->state == UrStateWaitHost) app->state = UrStateSpectrum;
                ur_send_config(app);
            }
        }
        break;
    case UR_MSG_SPECTRUM:
        if(len >= 1 + UR_SPECTRUM_BINS && app->connected) {
            memcpy(app->bins, &payload[1], UR_SPECTRUM_BINS);
            app->have_spectrum = true;
        }
        break;
    case UR_MSG_PULSES:
        if(len >= 1 && app->state == UrStateCapture) {
            uint8_t count = payload[0];
            if(count <= UR_PULSES_MAX && len >= 1 + count * sizeof(int32_t)) {
                int32_t durations[UR_PULSES_MAX];
                memcpy(durations, &payload[1], count * sizeof(int32_t));
                ur_capture_append(app, durations, count);
            }
        }
        break;
    case UR_MSG_STATUS:
        if(len >= 1 && payload[0] != UR_STATUS_OK) {
            char text[40] = "SDR error";
            if(len > 1) {
                size_t n = len - 1;
                if(n > sizeof(text) - 1) n = sizeof(text) - 1;
                memcpy(text, &payload[1], n);
                text[n] = '\0';
            }
            ur_notice(app, "%s", text);
        }
        break;
    default:
        break;
    }

    furi_mutex_release(app->mutex);
    view_port_update(app->view_port);
}

typedef enum {
    ParseMagic1,
    ParseMagic2,
    ParseType,
    ParseLen,
    ParsePayload,
    ParseChk,
} ParseState;

static int32_t ur_worker(void* context) {
    UsbRadioApp* app = context;

    ParseState ps = ParseMagic1;
    uint8_t ftype = 0, flen = 0, fpos = 0;
    uint8_t fpayload[256];

    while(true) {
        uint32_t events =
            furi_thread_flags_wait(WorkerEvtStop | WorkerEvtCdcRx, FuriFlagWaitAny, FuriWaitForever);
        if(events & WorkerEvtStop) break;
        if(!(events & WorkerEvtCdcRx)) continue;

        /* Drain the CDC endpoint, then parse byte by byte. */
        uint8_t chunk[CDC_DATA_SZ];
        int32_t n;
        while((n = furi_hal_cdc_receive(CDC_IF_NUM, chunk, sizeof(chunk))) > 0) {
            for(int32_t i = 0; i < n; i++) {
                uint8_t b = chunk[i];
                switch(ps) {
                case ParseMagic1:
                    if(b == UR_MAGIC1) ps = ParseMagic2;
                    break;
                case ParseMagic2:
                    ps = (b == UR_MAGIC2) ? ParseType : ParseMagic1;
                    break;
                case ParseType:
                    ftype = b;
                    ps = ParseLen;
                    break;
                case ParseLen:
                    flen = b;
                    fpos = 0;
                    ps = flen ? ParsePayload : ParseChk;
                    break;
                case ParsePayload:
                    fpayload[fpos++] = b;
                    if(fpos == flen) ps = ParseChk;
                    break;
                case ParseChk: {
                    uint8_t chk = ftype ^ flen;
                    for(uint8_t j = 0; j < flen; j++) chk ^= fpayload[j];
                    if(chk == b) {
                        ur_handle_frame(app, ftype, fpayload, flen);
                    } else {
                        FURI_LOG_W(TAG, "Bad checksum, frame type 0x%02X", ftype);
                    }
                    ps = ParseMagic1;
                    break;
                }
                }
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* CDC callbacks (USB IRQ context — keep them tiny)                    */
/* ------------------------------------------------------------------ */

static void cdc_on_tx_done(void* context) {
    UsbRadioApp* app = context;
    furi_semaphore_release(app->tx_done);
}

static void cdc_on_rx(void* context) {
    UsbRadioApp* app = context;
    furi_thread_flags_set(furi_thread_get_id(app->worker), WorkerEvtCdcRx);
}

static void cdc_on_ctrl_line(void* context, CdcCtrlLine ctrl_lines) {
    UsbRadioApp* app = context;
    app->dtr = (ctrl_lines & CdcCtrlLineDTR) != 0;
}

static CdcCallbacks cdc_callbacks = {
    .tx_ep_callback = cdc_on_tx_done,
    .rx_ep_callback = cdc_on_rx,
    .state_callback = NULL,
    .ctrl_line_callback = cdc_on_ctrl_line,
    .config_callback = NULL,
};

/* ------------------------------------------------------------------ */
/* Drawing                                                             */
/* ------------------------------------------------------------------ */

static void draw_freq(Canvas* canvas, uint8_t x, uint8_t y, uint32_t freq_hz, Align h_align) {
    char buf[20];
    snprintf(
        buf,
        sizeof(buf),
        "%lu.%03lu MHz",
        (unsigned long)(freq_hz / 1000000UL),
        (unsigned long)((freq_hz % 1000000UL) / 1000UL));
    canvas_draw_str_aligned(canvas, x, y, h_align, AlignBottom, buf);
}

static void draw_wait_host(Canvas* canvas, UsbRadioApp* app) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 10, "USB Radio");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 127, 10, AlignRight, AlignBottom, "@musicandmyth");
    canvas_draw_line(canvas, 0, 13, 127, 13);
    canvas_draw_str(canvas, 2, 25, "Connect Flipper USB to a PC");
    canvas_draw_str(canvas, 2, 36, "with an SDR, then run:");
    canvas_draw_str(canvas, 2, 47, "python usbradio_bridge.py");
    canvas_draw_str_aligned(
        canvas,
        64,
        62,
        AlignCenter,
        AlignBottom,
        app->dtr ? "Port open, waiting for bridge..." : "Waiting for serial port...");
}

static void draw_spectrum(Canvas* canvas, UsbRadioApp* app) {
    canvas_set_font(canvas, FontSecondary);
    draw_freq(canvas, 0, 9, app->freq_hz, AlignLeft);
    char buf[24];
    snprintf(
        buf, sizeof(buf), "+/-%s %s", step_labels[app->step_idx], span_labels[app->span_idx]);
    canvas_draw_str_aligned(canvas, 127, 9, AlignRight, AlignBottom, buf);

    /* Spectrum area: y = 12..63, display window -95..-5 dBFS. */
    const int top = 12;
    const int bottom = 63;
    const int height = bottom - top;

    if(app->have_spectrum) {
        int peak_bin = 0;
        int peak_val = -1;
        for(int x = 0; x < UR_SPECTRUM_BINS; x++) {
            int db = (int)app->bins[x] - UR_DB_OFFSET;
            if((int)app->bins[x] > peak_val) {
                peak_val = app->bins[x];
                peak_bin = x;
            }
            int h = ((db + 95) * height) / 90;
            if(h < 0) h = 0;
            if(h > height) h = height;
            if(h > 0) canvas_draw_line(canvas, x, bottom, x, bottom - h);
        }
        /* Peak readout, e.g. "-42dB" */
        snprintf(buf, sizeof(buf), "%ddB", peak_val - UR_DB_OFFSET);
        canvas_draw_str_aligned(canvas, 127, top + 7, AlignRight, AlignBottom, buf);
        canvas_draw_dot(canvas, peak_bin, top);
    } else {
        canvas_draw_str_aligned(canvas, 64, 40, AlignCenter, AlignBottom, "Tuning...");
    }

    /* Center frequency marker. */
    for(int y = top; y <= bottom; y += 4) {
        canvas_draw_dot(canvas, UR_SPECTRUM_BINS / 2, y);
    }
}

static void draw_capture(Canvas* canvas, UsbRadioApp* app) {
    canvas_set_font(canvas, FontPrimary);
    /* Blinking REC dot */
    if((furi_get_tick() / 512) % 2) canvas_draw_disc(canvas, 6, 7, 3);
    canvas_draw_str(canvas, 13, 11, "Capturing OOK");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_line(canvas, 0, 13, 127, 13);

    draw_freq(canvas, 2, 26, app->freq_hz, AlignLeft);

    char buf[32];
    snprintf(buf, sizeof(buf), "Pulses: %lu", (unsigned long)app->cap_pulses);
    canvas_draw_str(canvas, 2, 38, buf);
    uint32_t secs = (furi_get_tick() - app->cap_start_tick) / furi_ms_to_ticks(1000);
    snprintf(buf, sizeof(buf), "Elapsed: %lus", (unsigned long)secs);
    canvas_draw_str(canvas, 2, 49, buf);

    canvas_draw_str_aligned(canvas, 64, 62, AlignCenter, AlignBottom, "OK or Back: stop & save");
}

static void draw_callback(Canvas* canvas, void* context) {
    UsbRadioApp* app = context;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);
    switch(app->state) {
    case UrStateWaitHost:
        draw_wait_host(canvas, app);
        break;
    case UrStateSpectrum:
        draw_spectrum(canvas, app);
        break;
    case UrStateCapture:
        draw_capture(canvas, app);
        break;
    }

    /* Transient notice banner (saved file, errors). */
    if(app->notice_until && furi_get_tick() < app->notice_until) {
        canvas_set_font(canvas, FontSecondary);
        int w = canvas_string_width(canvas, app->notice) + 6;
        if(w > 128) w = 128;
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, (128 - w) / 2, 20, w, 13);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_frame(canvas, (128 - w) / 2, 20, w, 13);
        canvas_draw_str_aligned(canvas, 64, 30, AlignCenter, AlignBottom, app->notice);
    }

    furi_mutex_release(app->mutex);
}

/* ------------------------------------------------------------------ */
/* Input                                                               */
/* ------------------------------------------------------------------ */

static void input_callback(InputEvent* event, void* context) {
    UsbRadioApp* app = context;
    furi_message_queue_put(app->input_queue, event, 0);
}

/* Returns false when the app should exit. */
static bool handle_input(UsbRadioApp* app, const InputEvent* event) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    bool keep_running = true;
    bool config_changed = false;

    if(event->key == InputKeyBack && event->type == InputTypeLong) {
        keep_running = false;
    } else if(app->state == UrStateSpectrum) {
        uint32_t step = step_table[app->step_idx];
        if(event->type == InputTypeShort || event->type == InputTypeRepeat) {
            switch(event->key) {
            case InputKeyLeft:
                app->freq_hz = (app->freq_hz > FREQ_MIN + step) ? app->freq_hz - step : FREQ_MIN;
                config_changed = true;
                break;
            case InputKeyRight:
                app->freq_hz =
                    (app->freq_hz < FREQ_MAX - step) ? app->freq_hz + step : FREQ_MAX;
                config_changed = true;
                break;
            case InputKeyUp:
                if(event->type == InputTypeShort)
                    app->step_idx = (app->step_idx + 1) % COUNT_OF(step_table);
                break;
            case InputKeyDown:
                if(event->type == InputTypeShort)
                    app->step_idx =
                        (app->step_idx + COUNT_OF(step_table) - 1) % COUNT_OF(step_table);
                break;
            case InputKeyOk:
                if(event->type == InputTypeShort && app->connected) {
                    if(ur_capture_start(app)) {
                        app->state = UrStateCapture;
                        config_changed = true;
                    }
                }
                break;
            case InputKeyBack:
                if(event->type == InputTypeShort) keep_running = false;
                break;
            default:
                break;
            }
        } else if(event->type == InputTypeLong && event->key == InputKeyOk) {
            app->span_idx = (app->span_idx + 1) % COUNT_OF(span_table);
            config_changed = true;
        }
    } else if(app->state == UrStateCapture) {
        if(event->type == InputTypeShort &&
           (event->key == InputKeyOk || event->key == InputKeyBack)) {
            ur_capture_stop(app);
            app->state = UrStateSpectrum;
            config_changed = true;
        }
    } else { /* UrStateWaitHost */
        if(event->type == InputTypeShort && event->key == InputKeyBack) keep_running = false;
    }

    furi_mutex_release(app->mutex);
    if(config_changed) ur_send_config(app);
    view_port_update(app->view_port);
    return keep_running;
}

/* Periodic housekeeping: pings while disconnected, link timeout. */
static void handle_tick(UsbRadioApp* app) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    uint32_t now = furi_get_tick();

    if(app->connected) {
        if(now - app->last_rx_tick > furi_ms_to_ticks(LINK_TIMEOUT_MS)) {
            FURI_LOG_W(TAG, "Bridge link lost");
            app->connected = false;
            app->have_spectrum = false;
            if(app->state == UrStateCapture) ur_capture_stop(app);
            app->state = UrStateWaitHost;
        }
    }
    furi_mutex_release(app->mutex);

    if(!app->connected) {
        ur_send_frame(app, UR_MSG_PING, NULL, 0);
    }
    view_port_update(app->view_port);
}

/* ------------------------------------------------------------------ */
/* App lifecycle                                                       */
/* ------------------------------------------------------------------ */

int32_t usb_radio_app(void* p) {
    UNUSED(p);
    UsbRadioApp* app = malloc(sizeof(UsbRadioApp));
    memset(app, 0, sizeof(UsbRadioApp));

    app->state = UrStateWaitHost;
    app->freq_hz = 433920000UL;
    app->step_idx = 3; /* 25k */
    app->span_idx = 2; /* 2M */

    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->input_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->tx_done = furi_semaphore_alloc(1, 1);
    app->cap_line = furi_string_alloc();
    app->storage = furi_record_open(RECORD_STORAGE);

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);
    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    /* Take over the USB virtual COM port from the CLI. */
    app->cli_vcp = furi_record_open(RECORD_CLI_VCP);
    cli_vcp_disable(app->cli_vcp);
    app->usb_prev = furi_hal_usb_get_config();
    furi_hal_usb_unlock();
    furi_check(furi_hal_usb_set_config(&usb_cdc_single, NULL));

    app->worker = furi_thread_alloc_ex("UsbRadioWorker", 2048, ur_worker, app);
    furi_thread_start(app->worker);

    furi_hal_cdc_set_callbacks(CDC_IF_NUM, &cdc_callbacks, app);

    app->running = true;
    InputEvent event;
    while(app->running) {
        FuriStatus status =
            furi_message_queue_get(app->input_queue, &event, furi_ms_to_ticks(PING_PERIOD_MS));
        if(status == FuriStatusOk) {
            if(!handle_input(app, &event)) app->running = false;
        } else {
            handle_tick(app);
        }
    }

    /* Tell the bridge we are going away, best effort. */
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    if(app->state == UrStateCapture) ur_capture_stop(app);
    app->state = UrStateWaitHost;
    furi_mutex_release(app->mutex);
    UrConfig idle_cfg = {.mode = UR_MODE_IDLE};
    ur_send_frame(app, UR_MSG_CONFIG, &idle_cfg, sizeof(idle_cfg));

    /* Hand USB back to the CLI. */
    furi_hal_cdc_set_callbacks(CDC_IF_NUM, NULL, NULL);
    furi_thread_flags_set(furi_thread_get_id(app->worker), WorkerEvtStop);
    furi_thread_join(app->worker);
    furi_thread_free(app->worker);
    furi_hal_usb_unlock();
    furi_hal_usb_set_config(app->usb_prev, NULL);
    cli_vcp_enable(app->cli_vcp);
    furi_record_close(RECORD_CLI_VCP);

    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_STORAGE);

    furi_string_free(app->cap_line);
    furi_semaphore_free(app->tx_done);
    furi_message_queue_free(app->input_queue);
    furi_mutex_free(app->mutex);
    free(app);
    return 0;
}
