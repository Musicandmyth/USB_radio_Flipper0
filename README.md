# USB Radio for Flipper Zero

Use a USB SDR (RTL-SDR, HackRF, Airspy, SDRplay, ...) as a radio module for
the Flipper Zero.

The Flipper Zero's USB port is device-only — it cannot host a USB SDR
directly. So this project uses a **bridge** architecture:

```
[Flipper Zero] --USB serial--> [PC running host/usbradio_bridge.py] --USB--> [SDR]
```

The Flipper app is the tuner and UI; the PC does the DSP and streams results
back over the Flipper's virtual COM port.

## Features

- **Live spectrum analyzer** — 128-bin FFT waterfall bars on the Flipper
  screen, tunable center frequency, step and span, way beyond the CC1101's
  sub-GHz bands (whatever your SDR covers, e.g. 24 MHz – 1.7 GHz for RTL-SDR).
- **OOK/ASK signal capture** — the bridge demodulates pulses (rtl_433-style
  adaptive threshold) and the Flipper writes a standard **Sub-GHz RAW `.sub`
  file** to the SD card. Captures in the 300–928 MHz bands can be replayed
  with the Flipper's own CC1101 radio — the SDR acts as a much more
  sensitive receiver front-end.

## Building the Flipper app

```sh
pip install ufbt
ufbt            # builds dist/usb_radio.fap
ufbt launch     # or: build, install to the Flipper over USB, and run it
```

You can also copy `dist/usb_radio.fap` to the SD card under `apps/Sub-GHz/`
with qFlipper. Built and tested against firmware API 87.x (official firmware).

## Host bridge setup (PC side)

```sh
cd host
pip install -r requirements.txt
```

For **RTL-SDR** you also need the `librtlsdr` library:

- Windows: install the [rtl-sdr release](https://ftp.osmocom.org/binaries/windows/rtl-sdr/)
  DLLs somewhere on `PATH` (or use the PothosSDR installer), and install the
  WinUSB driver for the dongle with [Zadig](https://zadig.akeo.ie/).
- Linux: `sudo apt install librtlsdr0` (or equivalent).
- macOS: `brew install librtlsdr`.

For **other SDRs** install SoapySDR with Python bindings (Windows: the
[PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/) installer) and run
with `--backend soapy`.

## Usage

1. Plug the Flipper into the PC over USB. Close qFlipper / anything else
   holding the Flipper's COM port.
2. On the Flipper: **Apps → Sub-GHz → USB Radio**. The app takes over the
   USB serial port (the CLI is restored when you exit).
3. On the PC:

   ```sh
   python host/usbradio_bridge.py                       # RTL-SDR, autodetect port
   python host/usbradio_bridge.py --port COM5
   python host/usbradio_bridge.py --backend soapy --soapy-args driver=hackrf
   ```

4. The Flipper switches to the spectrum view as soon as the bridge connects.

### Controls (spectrum view)

| Key | Action |
| --- | --- |
| ◀ / ▶ | tune down / up by the current step (hold to repeat) |
| ▲ / ▼ | cycle frequency step (1k … 10M) |
| OK short | start OOK capture at the current frequency |
| OK long | cycle span (250k … 10M; host picks nearest SDR sample rate) |
| Back | exit |

### Controls (capture)

OK or Back stops the capture and saves
`SD Card/subghz/usbr-YYMMDD-HHMMSS.sub`, which shows up in the regular
Sub-GHz app under *Saved* for replay.

## Repository layout

- `application.fam`, `usb_radio.c`, `usb_radio_proto.h` — the Flipper app
- `host/usbradio_bridge.py` — PC bridge (pyrtlsdr / SoapySDR backends)
- `usb_radio_proto.h` documents the framed serial protocol; the Python side
  mirrors it and must be kept in sync

## Limitations & roadmap

- RTL-SDR is receive-only; TX (replaying `.sub` files through a HackRF) is
  reserved in the protocol (`UrHello.flags` bit 0) but not implemented yet.
- Capture is OOK/ASK only — FSK demodulation would be a natural next step.
- The link runs over USB CDC, so the Flipper must stay tethered to the PC.
  A future option is the same protocol over the GPIO UART to a small
  USB-host board (e.g. Raspberry Pi Zero) for a portable rig.
