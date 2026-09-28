#!/usr/bin/env python3
"""
USB Radio host bridge — drives a USB SDR and feeds the Flipper Zero
"USB Radio" app over the Flipper's USB virtual COM port.

The Flipper is the tuner/UI; this program does the DSP:
  - spectrum mode: FFT power spectrum, 128 bins, streamed at the
    frame rate the Flipper asks for
  - capture mode: OOK/ASK envelope demodulation into pulse durations,
    streamed to the Flipper which writes a Sub-GHz RAW .sub file

Backends: RTL-SDR via pyrtlsdr (default), or anything SoapySDR supports
(HackRF, Airspy, SDRplay, LimeSDR, ...) via --backend soapy.

Usage:
    python usbradio_bridge.py                # autodetect Flipper + RTL-SDR
    python usbradio_bridge.py --port COM5
    python usbradio_bridge.py --backend soapy --soapy-args driver=hackrf

Protocol: see ../usb_radio_proto.h. Frame = F0 9D | type | len | payload | xor.
"""

import argparse
import struct
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports

# ----------------------------------------------------------------------
# Protocol constants (mirror usb_radio_proto.h)
# ----------------------------------------------------------------------

MAGIC1, MAGIC2 = 0xF0, 0x9D
PROTO_VERSION = 1
SPECTRUM_BINS = 128

MSG_PING = 0x01
MSG_CONFIG = 0x02
MSG_HELLO = 0x81
MSG_SPECTRUM = 0x82
MSG_STATUS = 0x83
MSG_PULSES = 0x84

MODE_IDLE, MODE_SPECTRUM, MODE_CAPTURE = 0, 1, 2
GAIN_AUTO = 0xFFFF
DB_OFFSET = 130
STATUS_OK, STATUS_SDR_ERROR = 0, 1
PULSES_MAX = 48

FLIPPER_VID_PID = (0x0483, 0x5740)

# ----------------------------------------------------------------------
# SDR backends
# ----------------------------------------------------------------------


class RtlSdrBackend:
    """RTL-SDR via pyrtlsdr. RX only, 24 MHz - 1.766 GHz."""

    RATES = [250_000, 1_024_000, 1_400_000, 1_800_000, 1_920_000, 2_048_000, 2_400_000, 2_560_000]
    FREQ_RANGE = (24_000_000, 1_766_000_000)

    def __init__(self):
        from rtlsdr import RtlSdr  # noqa: import here so soapy users don't need it

        self.sdr = RtlSdr()
        self.name = "RTL-SDR"

    def pick_rate(self, span_hz):
        for r in self.RATES:
            if r >= span_hz:
                return r
        return self.RATES[-1]

    def configure(self, freq_hz, sample_rate, gain_db_tenths):
        lo, hi = self.FREQ_RANGE
        if not lo <= freq_hz <= hi:
            raise ValueError(f"{freq_hz/1e6:.3f} MHz out of RTL-SDR range")
        self.sdr.sample_rate = sample_rate
        self.sdr.center_freq = freq_hz
        if gain_db_tenths == GAIN_AUTO:
            self.sdr.gain = "auto"
        else:
            self.sdr.gain = gain_db_tenths / 10.0

    def read_samples(self, n):
        # pyrtlsdr wants multiples of 512; returns normalized complex128
        n = (n + 511) // 512 * 512
        return self.sdr.read_samples(n)

    def close(self):
        try:
            self.sdr.close()
        except Exception:
            pass


class SoapyBackend:
    """Any SoapySDR device (HackRF, Airspy, SDRplay, LimeSDR, ...)."""

    def __init__(self, device_args=""):
        import SoapySDR
        from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32

        self._soapy = SoapySDR
        self._RX = SOAPY_SDR_RX
        self._CF32 = SOAPY_SDR_CF32
        self.sdr = SoapySDR.Device(device_args)
        self.name = self.sdr.getHardwareKey() or "SoapySDR"
        self.stream = None
        self._buf = None

    def pick_rate(self, span_hz):
        rates = sorted(
            r for r in self.sdr.listSampleRates(self._RX, 0) if r >= 250_000
        ) or [2_000_000]
        for r in rates:
            if r >= span_hz:
                return int(r)
        return int(rates[-1])

    def configure(self, freq_hz, sample_rate, gain_db_tenths):
        self._teardown_stream()
        self.sdr.setSampleRate(self._RX, 0, sample_rate)
        self.sdr.setFrequency(self._RX, 0, freq_hz)
        try:
            if gain_db_tenths == GAIN_AUTO:
                self.sdr.setGainMode(self._RX, 0, True)
            else:
                self.sdr.setGainMode(self._RX, 0, False)
                self.sdr.setGain(self._RX, 0, gain_db_tenths / 10.0)
        except Exception:
            pass  # not every driver has AGC
        self.stream = self.sdr.setupStream(self._RX, self._CF32)
        self.sdr.activateStream(self.stream)

    def _teardown_stream(self):
        if self.stream is not None:
            try:
                self.sdr.deactivateStream(self.stream)
                self.sdr.closeStream(self.stream)
            except Exception:
                pass
            self.stream = None

    def read_samples(self, n):
        if self._buf is None or len(self._buf) < n:
            self._buf = np.empty(n, dtype=np.complex64)
        got = 0
        while got < n:
            sr = self.sdr.readStream(self.stream, [self._buf[got:]], n - got, timeoutUs=500_000)
            if sr.ret > 0:
                got += sr.ret
            elif sr.ret == 0:
                continue
            else:
                raise IOError(f"SoapySDR readStream error {sr.ret}")
        return self._buf[:n]

    def close(self):
        self._teardown_stream()
        self.sdr = None


# ----------------------------------------------------------------------
# Serial link
# ----------------------------------------------------------------------


def find_flipper_port():
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) == FLIPPER_VID_PID or "flipper" in (p.description or "").lower():
            return p.device
    return None


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0, write_timeout=2)
        self._rx = bytearray()

    def send(self, msg_type, payload=b""):
        frame = bytes([MAGIC1, MAGIC2, msg_type, len(payload)]) + payload
        chk = msg_type ^ len(payload)
        for b in payload:
            chk ^= b
        self.ser.write(frame + bytes([chk]))

    def poll(self):
        """Yield (type, payload) for every complete valid frame available."""
        data = self.ser.read(4096)
        if data:
            self._rx += data
        frames = []
        while True:
            start = self._rx.find(bytes([MAGIC1, MAGIC2]))
            if start < 0:
                # keep a trailing 0xF0 in case its 0x9D is still in flight
                self._rx = self._rx[-1:] if self._rx.endswith(bytes([MAGIC1])) else bytearray()
                break
            if start:
                del self._rx[:start]
            if len(self._rx) < 4:
                break
            mtype, mlen = self._rx[2], self._rx[3]
            if len(self._rx) < 4 + mlen + 1:
                break
            payload = bytes(self._rx[4 : 4 + mlen])
            chk = mtype ^ mlen
            for b in payload:
                chk ^= b
            if chk == self._rx[4 + mlen]:
                frames.append((mtype, payload))
                del self._rx[: 4 + mlen + 1]
            else:
                del self._rx[:2]  # bad frame, resync
        return frames

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ----------------------------------------------------------------------
# DSP
# ----------------------------------------------------------------------


def spectrum_frame(samples, bins=SPECTRUM_BINS, fft_size=1024):
    """Welch-averaged power spectrum -> `bins` bytes (dBFS + DB_OFFSET)."""
    n_seg = len(samples) // fft_size
    x = samples[: n_seg * fft_size].reshape(n_seg, fft_size)
    window = np.hanning(fft_size)
    spec = np.fft.fft(x * window, axis=1)
    power = (np.abs(spec) ** 2).mean(axis=0) / (np.sum(window**2) * fft_size)
    power = np.fft.fftshift(power)
    # collapse FFT bins to display bins with max() so narrow signals survive
    per_bin = fft_size // bins
    power = power[: bins * per_bin].reshape(bins, per_bin).max(axis=1)
    db = 10.0 * np.log10(power + 1e-20)
    return np.clip(db + DB_OFFSET, 0, 255).astype(np.uint8)


class OokDemod:
    """Envelope detector with adaptive threshold; emits alternating signed
    pulse durations in microseconds (+mark / -space), rtl_433 style."""

    SMOOTH = 8          # samples of boxcar smoothing on the magnitude
    MIN_PULSE_US = 40   # merge runs shorter than this into their neighbors
    MAX_DUR_US = 1_000_000
    MIN_SNR = 3.0       # linear magnitude ratio required to call it a signal

    def __init__(self, sample_rate):
        self.us_per_sample = 1e6 / sample_rate
        self.level = False        # current carrier state
        self.run_samples = 0      # length of the current run
        self.started = False      # becomes True at the first mark
        self.noise = None

    def process(self, samples):
        mag = np.abs(samples).astype(np.float32)
        if self.SMOOTH > 1:
            kernel = np.ones(self.SMOOTH, dtype=np.float32) / self.SMOOTH
            mag = np.convolve(mag, kernel, mode="same")

        noise = float(np.median(mag))
        self.noise = noise if self.noise is None else 0.8 * self.noise + 0.2 * noise
        peak = float(np.percentile(mag, 99.5))

        if peak < self.noise * self.MIN_SNR:
            hot = np.zeros(len(mag), dtype=bool)  # nothing but noise
        else:
            threshold = self.noise + 0.4 * (peak - self.noise)
            hot = mag > threshold

        return self._runs_to_pulses(hot)

    def _runs_to_pulses(self, hot):
        pulses = []
        # run-length encode, continuing the run carried over from last chunk
        edges = np.flatnonzero(np.diff(hot))
        bounds = np.concatenate(([0], edges + 1, [len(hot)]))
        for i in range(len(bounds) - 1):
            length = int(bounds[i + 1] - bounds[i])
            lvl = bool(hot[bounds[i]])
            if lvl == self.level:
                self.run_samples += length
                continue
            self._emit(pulses, self.level, self.run_samples)
            self.level = lvl
            self.run_samples = length
        # keep the trailing run pending; but flush overlong idle spaces so
        # the Flipper's pulse counter keeps moving during long captures
        if self.run_samples * self.us_per_sample > self.MAX_DUR_US:
            self._emit(pulses, self.level, self.run_samples)
            self.run_samples = 0
        return pulses

    def _emit(self, pulses, level, run_samples):
        if run_samples <= 0:
            return
        dur = int(run_samples * self.us_per_sample)
        dur = max(1, min(dur, self.MAX_DUR_US))
        if not self.started:
            if not level:
                return  # don't record leading silence
            self.started = True
        if dur < self.MIN_PULSE_US and pulses:
            # glitch: fold into the previous pulse to keep signs alternating
            pulses[-1] += dur if pulses[-1] > 0 else -dur
            return
        signed = dur if level else -dur
        if pulses and (pulses[-1] > 0) == (signed > 0):
            pulses[-1] += signed
        else:
            pulses.append(signed)
        return


# ----------------------------------------------------------------------
# Bridge main loop
# ----------------------------------------------------------------------


class Bridge:
    def __init__(self, link, backend):
        self.link = link
        self.backend = backend
        self.mode = MODE_IDLE
        self.freq_hz = 0
        self.span_hz = 0
        self.gain = GAIN_AUTO
        self.fps = 15
        self.sample_rate = 0
        self.demod = None
        self.last_hello = 0.0
        self.next_frame = 0.0

    def send_hello(self):
        name = self.backend.name.encode()[:21]
        payload = struct.pack("<BB22s", PROTO_VERSION, 0, name)
        self.link.send(MSG_HELLO, payload)
        self.last_hello = time.monotonic()

    def send_status(self, code, text=""):
        self.link.send(MSG_STATUS, bytes([code]) + text.encode()[:39] + b"\x00")

    def handle_config(self, payload):
        mode, freq, span, gain, fps = struct.unpack("<BIIHB", payload[:12])
        print(
            f"[flipper] mode={('idle','spectrum','capture')[mode]} "
            f"freq={freq/1e6:.3f}MHz span={span/1e6:.3g}MHz fps={fps}"
        )
        self.mode, self.freq_hz, self.span_hz, self.gain = mode, freq, span, gain
        self.fps = max(1, min(fps or 15, 30))
        self.next_frame = 0.0
        if mode == MODE_IDLE:
            return
        try:
            if mode == MODE_SPECTRUM:
                self.sample_rate = self.backend.pick_rate(span)
            else:  # capture: ~1 Msps is plenty for OOK remotes
                self.sample_rate = self.backend.pick_rate(1_000_000)
                self.demod = OokDemod(self.sample_rate)
            self.backend.configure(freq, self.sample_rate, gain)
        except Exception as e:
            print(f"[sdr] config failed: {e}", file=sys.stderr)
            self.send_status(STATUS_SDR_ERROR, str(e))
            self.mode = MODE_IDLE

    def run(self):
        self.send_hello()
        print("[bridge] running, waiting for the Flipper app...")
        while True:
            for mtype, payload in self.link.poll():
                if mtype == MSG_PING:
                    self.send_hello()
                elif mtype == MSG_CONFIG and len(payload) >= 12:
                    self.handle_config(payload)

            if self.mode == MODE_SPECTRUM:
                self.step_spectrum()
            elif self.mode == MODE_CAPTURE:
                self.step_capture()
            else:
                if time.monotonic() - self.last_hello > 1.0:
                    self.send_hello()
                time.sleep(0.05)

    def step_spectrum(self):
        now = time.monotonic()
        if now < self.next_frame:
            time.sleep(min(self.next_frame - now, 0.02))
            return
        self.next_frame = max(self.next_frame + 1.0 / self.fps, now)
        try:
            samples = self.backend.read_samples(16384)
        except Exception as e:
            self.sdr_died(e)
            return
        bins = spectrum_frame(np.asarray(samples))
        seq = int(now * self.fps) & 0xFF
        self.link.send(MSG_SPECTRUM, bytes([seq]) + bins.tobytes())

    def step_capture(self):
        try:
            samples = self.backend.read_samples(32768)
        except Exception as e:
            self.sdr_died(e)
            return
        pulses = self.demod.process(np.asarray(samples))
        for i in range(0, len(pulses), PULSES_MAX):
            batch = pulses[i : i + PULSES_MAX]
            payload = bytes([len(batch)]) + struct.pack(f"<{len(batch)}i", *batch)
            self.link.send(MSG_PULSES, payload)

    def sdr_died(self, e):
        print(f"[sdr] read failed: {e}", file=sys.stderr)
        self.send_status(STATUS_SDR_ERROR, "SDR read failed")
        self.mode = MODE_IDLE


def main():
    ap = argparse.ArgumentParser(description="Flipper Zero USB Radio host bridge")
    ap.add_argument("--port", help="Flipper COM port (default: autodetect)")
    ap.add_argument(
        "--backend", choices=["rtlsdr", "soapy"], default="rtlsdr", help="SDR backend"
    )
    ap.add_argument(
        "--soapy-args", default="", help='SoapySDR device args, e.g. "driver=hackrf"'
    )
    args = ap.parse_args()

    port = args.port or find_flipper_port()
    if not port:
        sys.exit(
            "No Flipper serial port found. Plug the Flipper in, open the USB Radio "
            "app on it, and/or pass --port COMx explicitly."
        )

    print(f"[bridge] opening SDR ({args.backend})...")
    try:
        if args.backend == "rtlsdr":
            backend = RtlSdrBackend()
        else:
            backend = SoapyBackend(args.soapy_args)
    except Exception as e:
        sys.exit(
            f"Could not open SDR: {e}\n"
            "rtlsdr: pip install pyrtlsdr, and make sure librtlsdr + WinUSB driver "
            "(Zadig) are installed.\n"
            "soapy: install SoapySDR (e.g. PothosSDR) with Python bindings."
        )
    print(f"[bridge] SDR ready: {backend.name}")

    print(f"[bridge] opening Flipper on {port}...")
    link = Link(port)
    bridge = Bridge(link, backend)
    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\n[bridge] bye")
    finally:
        link.close()
        backend.close()


if __name__ == "__main__":
    main()
