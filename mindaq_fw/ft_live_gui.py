#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import csv
import math
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from glob import glob
from os import name as os_name

import serial

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit(
        "pyqtgraph and a Qt binding are required. Install pyqtgraph plus PyQt5/PyQt6/PySide6."
    ) from exc

BAUD = 2_000_000
SERIAL_PORT = None
FT_CHANNELS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
FT_UNITS = ["N", "N", "N", "Nm", "Nm", "Nm"]
SAMPLE_RATE_HZ = 32_000.0
DISPLAY_RATE_HZ = 1000.0
DISPLAY_DECIMATE = int(SAMPLE_RATE_HZ / DISPLAY_RATE_HZ)
MIN_DISPLAY_SAMPLES = 250
RAW_CHANNELS = 6
SAMPLES_PER_BLOCK = 64
RAW_BYTES = RAW_CHANNELS * 3
SYNC = 0xA55AA55A
SYNC_BYTES = b"\x5a\xa5\x5a\xa5"
BLOCK = struct.Struct(f"<II{SAMPLES_PER_BLOCK * RAW_BYTES}sBBBB")
BLOCK_SIZE = BLOCK.size
SERIAL_READ_SIZE = 65536
STARTUP_DRAIN_MIN_SECONDS = 0.35
STARTUP_DRAIN_MAX_SECONDS = 2.5
WINDOW_SECONDS = 5.0
COLORS = ["#ff6b35", "#f39c12", "#27ae60", "#2980b9", "#8e44ad", "#c0392b"]
NOTCH_Q = 10.0
NOTCH_FREQS = (60.0, 120.0)
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB", "COM")
GAIN_LABELS = ("x1", "x2", "x4", "x8", "x16", "x32")
DEFAULT_GAIN_CODE = 3
ADC_FULL_SCALE = 0x7FFFFF
# Measured no-load ADC means in ADC port order.
RAW_ZERO_CODE = (-995095.6, -358128.7, -940395.8, -265481.2, -836644.1, 18634.5)
# ADC microvolt deltas -> ATI XML gauge order [g0,g1,g2,g3,g4,g5].
# Nonzero locations encode wiring: port0->g4, port1->g5, port2->g2, port3->g3, port4->g0, port5->g1.
# Nonzero values include the working ADC-count-to-NetFT-gauge bridge and XML GaugeGains normalization.
ADC_UV_TO_ATI_GAUGE = (
    (0.0, 0.0, 0.0, 0.0, 1.059662393280649e+00, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 8.162949541379004e-01),
    (0.0, 0.0, 9.272045943442637e-01, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 9.322712858379332e-01, 0.0, 0.0),
    (8.363021832919449e-01, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 8.404218980265711e-01, 0.0, 0.0, 0.0, 0.0),
)
# ATI XML gauge order -> [Fx,Fy,Fz,Tx,Ty,Tz]. Only Fz is fitted; other rows are factory XML.
ATI_GAUGE_TO_FT = (
    (6.054530989079390e-06, 2.075418832354940e-05, -9.575586391247820e-06,
     -5.034719323341890e-04, -2.827122413565440e-05, 5.179287349565690e-04),
    (-2.520127206219610e-05, 6.348497973460170e-04, -1.038902452683050e-07,
     -2.775572866673270e-04, 8.796242351000440e-06, -3.247148267647300e-04),
    (4.838318054700e-04, 0.0, 5.434076606576e-04, 0.0, 5.993160239162e-04, 0.0),
    (-2.577052836577890e-07, 3.830766465429980e-06, 3.124957530622520e-06,
     -1.381712094274260e-06, -3.204207898418680e-06, -2.084386424212550e-06),
    (-3.172020201011310e-06, -4.085733997308220e-07, 1.741821139441800e-06,
     3.219636663939340e-06, 2.229410518476670e-06, -3.070929538864320e-06),
    (-9.943845273683800e-08, 2.240688262268020e-06, 2.397902398001370e-08,
     2.162107308175090e-06, -7.705459896268520e-08, 2.297459226084220e-06),
)


def count_to_microvolt(gain_code: int) -> float:
    return (1.2e6 / float(1 << gain_code)) / 8388608.0


RAW_ZERO_MICROVOLT = tuple(value * count_to_microvolt(DEFAULT_GAIN_CODE) for value in RAW_ZERO_CODE)


def plot_position(index: int) -> tuple[int, int]:
    return index % 3, index // 3


def find_serial_port(pattern: str | None) -> str | None:
    if pattern:
        if os_name == "nt" and pattern.upper().startswith("COM"):
            return pattern
        if list_ports is not None:
            for port_info in list_ports.comports():
                if port_info.device == pattern:
                    return pattern
        matches = sorted(glob(pattern))
        if matches:
            for match in matches:
                if match.startswith(USB_PORT_PREFIXES):
                    return match
            return None
        for prefix in USB_PORT_PREFIXES:
            if pattern.startswith(prefix):
                family = sorted(glob(f"{prefix}*"))
                if family:
                    return family[0]
        return None

    if list_ports is not None:
        ports = list(list_ports.comports())

        def score(port_info) -> tuple[int, str]:
            device = port_info.device or ""
            if not device.startswith(USB_PORT_PREFIXES):
                return -1, device
            text = " ".join(
                [
                    device,
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            if "bluetooth" in text:
                return -1, device
            value = 0
            if getattr(port_info, "vid", None) == 0x303A:
                value += 100
            if "esp32" in text or "espressif" in text:
                value += 80
            if "usbmodem" in text or "cdc" in text:
                value += 20
            return value, device

        if ports:
            ranked = [item for item in ports if score(item)[0] >= 0]
            if ranked:
                return sorted(ranked, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for candidate in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(candidate))
        if matches:
            return matches[0]

    return None


def available_serial_ports() -> list[str]:
    ports: list[str] = []
    seen: set[str] = set()

    def add(device: str) -> None:
        if not device.startswith(USB_PORT_PREFIXES) or device in seen:
            return
        seen.add(device)
        ports.append(device)

    if list_ports is not None:
        for port_info in sorted(list_ports.comports(), key=lambda item: item.device or ""):
            device = port_info.device or ""
            text = " ".join(
                [
                    device,
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            if "bluetooth" in text:
                continue
            add(device)

    for candidate in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for match in sorted(glob(candidate)):
            add(match)

    return ports


def block_valid(frame: bytes) -> bool:
    check = 0
    for byte in frame[:-1]:
        check ^= byte
    return check == frame[-1]


def raw24(data: bytes, index: int) -> int:
    base = index * 3
    value = data[base] | (data[base + 1] << 8) | (data[base + 2] << 16)
    if value & 0x800000:
        value |= ~0xFFFFFF
    return value


def resolve_ft(raw: tuple[int, ...], gain_code: int = DEFAULT_GAIN_CODE) -> tuple[float, ...]:
    adc_uv = tuple(float(value) * count_to_microvolt(gain_code) - RAW_ZERO_MICROVOLT[index]
                   for index, value in enumerate(raw))
    ati_gage = tuple(sum(row[col] * adc_uv[col] for col in range(RAW_CHANNELS))
                    for row in ADC_UV_TO_ATI_GAUGE)
    return tuple(sum(row[col] * ati_gage[col] for col in range(RAW_CHANNELS))
                 for row in ATI_GAUGE_TO_FT)


@dataclass
class Sample:
    timestamp_s: float
    seq: int
    gain_code: int
    warn_flags: int
    clip_flags: int
    raw: tuple[int, ...]
    ft: tuple[float, ...]


class SampleBuffer:
    def __init__(self, seconds: float = WINDOW_SECONDS, rate_hz: float = SAMPLE_RATE_HZ) -> None:
        self.samples: deque[Sample] = deque(maxlen=max(1000, int(seconds * rate_hz)))
        self.lock = threading.Lock()

    def append(self, sample: Sample) -> None:
        with self.lock:
            self.samples.append(sample)

    def snapshot(self) -> list[Sample]:
        with self.lock:
            return list(self.samples)

    def display_snapshot(self) -> list[Sample]:
        with self.lock:
            return list(self.samples)[::DISPLAY_DECIMATE]

    def clear(self) -> None:
        with self.lock:
            self.samples.clear()


class NotchFilter:
    def __init__(self, freq_hz: float, sample_rate_hz: float = DISPLAY_RATE_HZ, q: float = NOTCH_Q) -> None:
        w0 = 2.0 * math.pi * freq_hz / sample_rate_hz
        alpha = math.sin(w0) / (2.0 * q)
        a0 = 1.0 + alpha
        self.b0 = 1.0 / a0
        self.b1 = (-2.0 * math.cos(w0)) / a0
        self.b2 = 1.0 / a0
        self.a1 = (-2.0 * math.cos(w0)) / a0
        self.a2 = (1.0 - alpha) / a0
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0
        self.ready = False

    def step(self, value: float) -> float:
        if not self.ready:
            self.x1 = value
            self.x2 = value
            self.y1 = value
            self.y2 = value
            self.ready = True
            return value
        output = (
            self.b0 * value
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2
        )
        self.x2 = self.x1
        self.x1 = value
        self.y2 = self.y1
        self.y1 = output
        return output


class SerialReader(threading.Thread):
    def __init__(self, port_hint: str | None, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port_hint = port_hint
        self.samples = samples
        self.messages = messages
        self.stop_event = threading.Event()
        self.ser: serial.Serial | None = None
        self.current_port = port_hint
        self.buffer = bytearray()
        self.last_seq: int | None = None
        self.missing = 0
        self.gain_code = DEFAULT_GAIN_CODE
        self.ft_filters = [
            [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS
        ]
        self.connected = False
        self.port_lock = threading.Lock()

    def reset_filters(self) -> None:
        self.ft_filters = [[NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS]

    def reset_stream(self) -> None:
        self.samples.clear()
        self.buffer.clear()
        self.last_seq = None
        self.missing = 0
        self.reset_filters()

    def connect(self) -> bool:
        with self.port_lock:
            port_hint = self.port_hint
        port = find_serial_port(port_hint)
        if port is None:
            if self.connected:
                self.messages.put("searching for board")
            self.connected = False
            self.current_port = port_hint
            return False
        try:
            self.ser = open_serial(port)
        except (serial.SerialException, OSError):
            self.ser = None
            self.connected = False
            self.current_port = port
            return False
        self.current_port = port
        self.connected = True
        self.messages.put(f"draining {port}")
        try:
            self.drain_startup_data()
        except (serial.SerialException, OSError):
            self.disconnect()
            return False
        self.reset_stream()
        self.messages.put(f"connected {port}")
        return True

    def drain_startup_data(self) -> None:
        if self.ser is None:
            return
        start = time.monotonic()
        deadline = start + STARTUP_DRAIN_MAX_SECONDS
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            chunk = self.ser.read(SERIAL_READ_SIZE)
            elapsed = time.monotonic() - start
            if elapsed >= STARTUP_DRAIN_MIN_SECONDS and len(chunk) < SERIAL_READ_SIZE:
                break
        self.ser.reset_input_buffer()
        self.buffer.clear()

    def disconnect(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        if self.connected and not self.stop_event.is_set():
            self.messages.put("reconnecting")
        self.connected = False

    def run(self) -> None:
        while not self.stop_event.is_set():
            if self.ser is None and not self.connect():
                time.sleep(0.5)
                continue
            try:
                chunk = self.ser.read(SERIAL_READ_SIZE)
            except (serial.SerialException, OSError) as exc:
                if self.stop_event.is_set():
                    break
                self.messages.put(f"[serial error] {exc}")
                self.disconnect()
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            self.buffer.extend(chunk)
            self._parse_buffer()

    def _parse_buffer(self) -> None:
        while len(self.buffer) >= BLOCK_SIZE:
            if self.buffer[:4] != SYNC_BYTES:
                index = self.buffer.find(SYNC_BYTES, 1)
                if index < 0:
                    del self.buffer[:-3]
                    break
                del self.buffer[:index]
                continue

            frame = bytes(self.buffer[:BLOCK_SIZE])
            if not block_valid(frame):
                del self.buffer[0]
                continue

            sync, seq, raw_block, gain_code, warn_flags, clip_flags, _checksum = BLOCK.unpack(frame)
            if sync == SYNC:
                self.gain_code = gain_code
                if self.last_seq is not None:
                    expected = self.last_seq + 1
                    if seq < expected:
                        self.samples.clear()
                        self.reset_filters()
                        self.missing = 0
                    elif seq > expected:
                        self.missing += seq - expected
                for offset in range(SAMPLES_PER_BLOCK):
                    sample_seq = seq + offset
                    start = offset * RAW_BYTES
                    raw_bytes = raw_block[start:start + RAW_BYTES]
                    raw = tuple(raw24(raw_bytes, index) for index in range(RAW_CHANNELS))
                    self.samples.append(
                        Sample(
                            timestamp_s=sample_seq / SAMPLE_RATE_HZ,
                            seq=sample_seq,
                            gain_code=gain_code,
                            warn_flags=warn_flags,
                            clip_flags=clip_flags,
                            raw=raw,
                            ft=resolve_ft(raw, gain_code),
                        )
                    )
                self.last_seq = seq + SAMPLES_PER_BLOCK - 1

            del self.buffer[:BLOCK_SIZE]

    def stop(self) -> None:
        self.stop_event.set()
        self.disconnect()

    def set_port_hint(self, port_hint: str | None) -> None:
        with self.port_lock:
            self.port_hint = port_hint
            self.current_port = port_hint
        self.disconnect()

    def set_gain_code(self, gain_code: int) -> bool:
        if gain_code < 0 or gain_code >= len(GAIN_LABELS):
            return False
        if self.ser is None:
            self.messages.put("gain command needs connected board")
            return False
        try:
            self.ser.write(f"gain {gain_code}\n".encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            self.messages.put(f"[serial error] {exc}")
            self.disconnect()
            return False
        return True


class PlotWindow:
    def __init__(self, reader: SerialReader, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.reader = reader
        self.samples = samples
        self.messages = messages
        self.last_message = "connecting"
        self.paused = False
        self.paused_samples: list[Sample] | None = None
        self.filter_ft = False
        self.zero_ft = [0.0] * len(FT_CHANNELS)
        self.auto_zero_pending = True
        self.display_filters = [
            [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS
        ]

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq force/torque live")
        self.win.resize(1400, 900)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel("port=searching | waiting for telemetry")
        layout.addWidget(self.status)

        button_row = QtWidgets.QHBoxLayout()
        self.port_box = QtWidgets.QComboBox()
        self.port_box.currentIndexChanged.connect(self.change_port)
        button_row.addWidget(self.port_box)

        self.refresh_ports_button = QtWidgets.QPushButton("Refresh Ports")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        button_row.addWidget(self.refresh_ports_button)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        button_row.addWidget(self.pause_button)

        self.zero_button = QtWidgets.QPushButton("Zero All")
        self.zero_button.clicked.connect(self.zero_all)
        button_row.addWidget(self.zero_button)

        self.filter_button = QtWidgets.QPushButton("60 Hz Filter Off")
        self.filter_button.clicked.connect(self.toggle_filter)
        button_row.addWidget(self.filter_button)

        button_row.addWidget(QtWidgets.QLabel("Gain"))
        self.gain_box = QtWidgets.QComboBox()
        for code, label in enumerate(GAIN_LABELS):
            self.gain_box.addItem(label, code)
        self.gain_box.setCurrentIndex(DEFAULT_GAIN_CODE)
        self.gain_box.currentIndexChanged.connect(self.change_gain)
        button_row.addWidget(self.gain_box)

        self.save_button = QtWidgets.QPushButton("Save CSV")
        self.save_button.clicked.connect(self.save_csv)
        button_row.addWidget(self.save_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self.grid = QtWidgets.QGridLayout()
        layout.addLayout(self.grid)

        self.curves: list[pg.PlotDataItem] = []
        self.plots: list[pg.PlotWidget] = []
        for index, name in enumerate(FT_CHANNELS):
            plot = pg.PlotWidget(title=name)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", FT_UNITS[index])
            plot.setLabel("bottom", "Time (s)")
            curve = plot.plot(pen=pg.mkPen(COLORS[index], width=2))
            self.plots.append(plot)
            self.curves.append(curve)
            row, col = plot_position(index)
            self.grid.addWidget(plot, row, col)

        self.win.show()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

        self.refresh_ports()

    def current_names(self) -> list[str]:
        return FT_CHANNELS

    def current_unit(self, index: int) -> str:
        return FT_UNITS[index]

    def sample_values(self, sample: Sample) -> tuple[float, ...]:
        return sample.ft

    def display_values(self, samples: list[Sample]) -> list[tuple[float, ...]]:
        if not self.filter_ft:
            return [sample.ft for sample in samples]
        self.display_filters = [
            [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS
        ]
        values: list[tuple[float, ...]] = []
        for sample in samples:
            filtered = list(sample.ft)
            for index, value in enumerate(filtered):
                for filt in self.display_filters[index]:
                    value = filt.step(value)
                filtered[index] = value
            values.append(tuple(filtered))
        return values

    def zero_values(self) -> list[float]:
        return self.zero_ft

    def refresh_ports(self) -> None:
        current = self.port_box.currentData()
        if current is None:
            current = self.reader.port_hint
        ports = available_serial_ports()

        self.port_box.blockSignals(True)
        self.port_box.clear()
        self.port_box.addItem("Auto", None)
        for port in ports:
            self.port_box.addItem(port, port)

        index = self.port_box.findData(current)
        if index < 0 and self.reader.current_port is not None:
            index = self.port_box.findData(self.reader.current_port)
        self.port_box.setCurrentIndex(index if index >= 0 else 0)
        self.port_box.blockSignals(False)

    def change_port(self) -> None:
        port_hint = self.port_box.currentData()
        self.reader.set_port_hint(port_hint)
        self.auto_zero_pending = True
        self.last_message = f"port={'auto' if port_hint is None else port_hint}"

    def change_gain(self) -> None:
        gain_code = self.gain_box.currentData()
        if gain_code is None:
            return
        self.reader.set_gain_code(int(gain_code))
        self.samples.clear()
        self.reader.reset_filters()
        self.display_filters = [
            [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS
        ]
        self.paused_samples = None
        self.auto_zero_pending = False
        self.last_message = "gain changed; re-zero if needed"

    def toggle_filter(self) -> None:
        self.filter_ft = not self.filter_ft
        self.filter_button.setText("60 Hz Filter On" if self.filter_ft else "60 Hz Filter Off")
        self.last_message = "60/120 Hz notch on" if self.filter_ft else "60/120 Hz notch off"

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.paused_samples = self.samples.snapshot() if self.paused else None
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.last_message = "plot paused" if self.paused else "plot resumed"

    def zero_all(self) -> None:
        samples = self.samples.snapshot()
        if not samples:
            self.last_message = "no sample to zero"
            return
        values = self.sample_values(samples[-1])
        zero = self.zero_values()
        for index, value in enumerate(values):
            zero[index] = value
        self.last_message = "zero set"

    def auto_zero(self, samples: list[Sample]) -> None:
        latest = samples[-1]
        self.zero_ft = list(self.sample_values(latest))
        self.auto_zero_pending = False
        self.last_message = "startup zero set"

    def save_csv(self) -> None:
        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        if not samples:
            self.last_message = "no samples to save"
            return

        default_name = f"mindaq_ft_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.win,
            "Save CSV",
            default_name,
            "CSV Files (*.csv)",
        )
        if not path:
            return

        header = ["timestamp_s"]
        header.append("seq")
        header.extend(["gain_code", "warn_flags", "clip_flags"])
        header.extend(f"adc_{index}" for index in range(RAW_CHANNELS))
        header.extend(f"{name}_raw" for name in FT_CHANNELS)
        header.extend(f"{name}_zeroed" for name in FT_CHANNELS)

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for sample in samples:
                writer.writerow(
                    [f"{sample.timestamp_s:.8f}", sample.seq]
                    + [sample.gain_code, sample.warn_flags, sample.clip_flags]
                    + list(sample.raw)
                    + [f"{value:.3f}" for value in sample.ft]
                    + [f"{value - zero:.3f}" for value, zero in zip(sample.ft, self.zero_ft)]
                )

        self.last_message = f"saved {len(samples)} samples"

    def tick(self) -> None:
        while True:
            try:
                self.last_message = self.messages.get_nowait()
            except queue.Empty:
                break

        if self.paused_samples is not None:
            samples = self.paused_samples[::DISPLAY_DECIMATE]
        else:
            samples = self.samples.display_snapshot()
        port = self.reader.current_port or "searching"
        if len(samples) < MIN_DISPLAY_SAMPLES:
            for curve in self.curves:
                curve.setData([], [])
            self.status.setText(
                f"port={port} | loading real stream {len(samples)}/{MIN_DISPLAY_SAMPLES} | {self.last_message}"
            )
            return

        if self.auto_zero_pending:
            self.auto_zero(samples)
            self.status.setText(f"port={port} | {self.last_message}")
            return

        if self.last_message == "connecting":
            self.last_message = "streaming"

        start_s = samples[0].timestamp_s
        xs = [sample.timestamp_s - start_s for sample in samples]
        latest = samples[-1]
        zero = self.zero_values()
        values = self.display_values(samples)

        for index, curve in enumerate(self.curves):
            ys = [value[index] - zero[index] for value in values]
            curve.setData(xs, ys)

        rate_hz = 0.0
        if len(samples) > 8:
            window = samples[-256:] if len(samples) > 256 else samples
            dt = window[-1].timestamp_s - window[0].timestamp_s
            if dt > 0:
                rate_hz = (len(window) - 1) / dt

        latest_values = values[-1]
        latest_text = " ".join(
            f"{name}={value - offset:+.3f}{self.current_unit(index)}"
            for index, (name, value, offset) in enumerate(zip(self.current_names(), latest_values, zero))
        )
        warn_flags = 0
        clip_flags = 0
        headroom = [0.0] * RAW_CHANNELS
        for sample in samples:
            warn_flags |= sample.warn_flags
            clip_flags |= sample.clip_flags
            for index, value in enumerate(sample.raw):
                headroom[index] = max(headroom[index], abs(value) * 100.0 / ADC_FULL_SCALE)
        sat_parts = []
        if clip_flags:
            sat_parts.append("SAT CLIP " + ",".join(f"ch{i}" for i in range(RAW_CHANNELS) if clip_flags & (1 << i)))
        warn_only = warn_flags & ~clip_flags
        if warn_only:
            sat_parts.append("SAT WARN " + ",".join(f"ch{i}" for i in range(RAW_CHANNELS) if warn_only & (1 << i)))
        sat_text = " | " + " ".join(sat_parts) if sat_parts else ""
        raw_pct = "/".join(f"{value:.0f}" for value in headroom)
        self.status.setText(
            f"port={port} | gain={GAIN_LABELS[latest.gain_code]} raw%={raw_pct}{sat_text} | "
            f"plot={rate_hz:.1f} Hz save=32 kSPS missing={self.reader.missing} | {latest_text} | {self.last_message}"
        )

    def exec(self) -> int:
        return self.app.exec()


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.05)
    ser.dtr = True
    ser.rts = False
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=SERIAL_PORT)
    args = parser.parse_args()

    samples = SampleBuffer()
    messages: queue.Queue[str] = queue.Queue()
    reader = SerialReader(args.port, samples, messages)
    reader.start()

    window = PlotWindow(reader, samples, messages)
    try:
        return window.exec()
    finally:
        reader.stop()
        reader.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
