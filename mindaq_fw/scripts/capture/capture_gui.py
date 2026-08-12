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
from pathlib import Path

import numpy as np
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
SAMPLE_RATE_HZ = 32_000.0
DISPLAY_RATE_HZ = 1000.0
DISPLAY_DECIMATE = int(SAMPLE_RATE_HZ / DISPLAY_RATE_HZ)
MIN_DISPLAY_SAMPLES = 250
CHANNEL_NAMES = ("ADC 0", "ADC 1", "ADC 2", "ADC 3", "ADC 4", "ADC 5", "Isense", "Vsense")
CHANNELS = len(CHANNEL_NAMES)
SAMPLES_PER_BLOCK = 64
RAW_BYTES = CHANNELS * 3
SYNC = 0xA55AA55A
SYNC_BYTES = b"\x5a\xa5\x5a\xa5"
BLOCK = struct.Struct(f"<II{SAMPLES_PER_BLOCK * RAW_BYTES}sBBBB")
BLOCK_SIZE = BLOCK.size
SERIAL_READ_SIZE = 65536
STARTUP_DRAIN_MIN_SECONDS = 0.35
STARTUP_DRAIN_MAX_SECONDS = 2.5
WINDOW_SECONDS = 5.0
BUFFER_SECONDS = 5.0
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB", "COM")
GAIN_LABELS = ("x1", "x2", "x4", "x8", "x16", "x32", "x64", "x128")
DEFAULT_GAIN_CODE = 7
ADC_FULL_SCALE = 0x7FFFFF
TRIGGER_PINS = (43, 44, 1, 2, 3, 4, 5, 6, 7)
DEFAULT_TRIGGER_LEVEL_UV = 100.0
DEFAULT_TRIGGER_DELAY_MS = 200.0
DEFAULT_CAPTURE_WINDOW_START_MS = -500.0
DEFAULT_CAPTURE_WINDOW_END_MS = 0.0
DEFAULT_UV_PER_N = 101.3232126
TRIGGER_LOCKOUT_SECONDS = 1.0
TRIGGER_LOCKOUT_SAMPLES = int(TRIGGER_LOCKOUT_SECONDS * SAMPLE_RATE_HZ)
TRIGGER_SLIDER_STEPS = 10000
DEFAULT_FIR_CUTOFF_HZ = 100.0
MAX_FIR_CUTOFF_HZ = DISPLAY_RATE_HZ * 0.45
FIR_TAPS = 63
DEFAULT_POS_CHANNEL = 1
DEFAULT_NEG_CHANNEL = 0


def count_to_microvolt(gain_code: int) -> float:
    return (1.2e6 / float(1 << gain_code)) / 8388608.0


def sample_rate(samples: list["Sample"]) -> float:
    if len(samples) < 2:
        return DISPLAY_RATE_HZ
    seq_span = samples[-1].seq - samples[0].seq
    if seq_span <= 0:
        return DISPLAY_RATE_HZ
    seq_step = max(1, int(round(seq_span / (len(samples) - 1))))
    return SAMPLE_RATE_HZ / seq_step


def fir_coefficients(cutoff_hz: float, rate_hz: float) -> np.ndarray:
    cutoff_hz = max(1.0, min(cutoff_hz, rate_hz * 0.49))
    norm = cutoff_hz / rate_hz
    index = np.arange(FIR_TAPS, dtype=float)
    offset = index - (FIR_TAPS - 1) / 2.0
    window = 0.54 - 0.46 * np.cos(2.0 * math.pi * index / (FIR_TAPS - 1))
    values = 2.0 * norm * np.sinc(2.0 * norm * offset) * window
    return values / values.sum()


def apply_fir(values: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    if len(values) < 2:
        return values
    half = len(coefficients) // 2
    padded = np.pad(values, (half, half), mode="edge")
    return np.correlate(padded, coefficients, mode="valid")


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
                [device, port_info.description or "", port_info.manufacturer or "", port_info.hwid or ""]
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
                [device, port_info.description or "", port_info.manufacturer or "", port_info.hwid or ""]
            ).lower()
            if "bluetooth" not in text:
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


def raw_to_uv(raw: tuple[int, ...], gain_code: int) -> tuple[float, ...]:
    scale = count_to_microvolt(gain_code)
    return tuple(value * scale for value in raw)


def channel_text(pos: int | None, neg: int | None) -> str:
    if pos is None and neg is None:
        return "None"
    if pos is None:
        return CHANNEL_NAMES[neg]
    if neg is None:
        return CHANNEL_NAMES[pos]
    return f"{CHANNEL_NAMES[pos]} - {CHANNEL_NAMES[neg]}"


def channel_value(values: tuple[float, ...], pos: int | None, neg: int | None) -> float:
    if pos is None and neg is None:
        return 0.0
    if pos is None:
        return values[neg]
    if neg is None:
        return values[pos]
    return values[pos] - values[neg]


@dataclass
class Sample:
    timestamp_s: float
    seq: int
    gain_code: int
    warn_flags: int
    clip_flags: int
    raw: tuple[int, ...]
    uv: tuple[float, ...]


@dataclass
class TriggerTrace:
    samples: list[Sample]
    cross_seq: float
    cross_uv: float
    pulse_start_seq: float
    pulse_end_seq: float


@dataclass
class PendingTrigger:
    cross_seq: float
    cross_uv: float
    pulse_start_seq: float
    pulse_end_seq: float
    pre_seq: int
    post_seq: int


class SampleBuffer:
    def __init__(self, seconds: float = BUFFER_SECONDS, rate_hz: float = SAMPLE_RATE_HZ) -> None:
        self.samples: deque[Sample] = deque(maxlen=max(1000, int(seconds * rate_hz)))
        self.display_samples: deque[Sample] = deque(maxlen=max(1000, int(WINDOW_SECONDS * DISPLAY_RATE_HZ)))
        self.lock = threading.Lock()

    def append(self, sample: Sample) -> None:
        with self.lock:
            self.samples.append(sample)
            if sample.seq % DISPLAY_DECIMATE == 0:
                self.display_samples.append(sample)

    def snapshot(self) -> list[Sample]:
        with self.lock:
            return list(self.samples)

    def display_snapshot(self) -> list[Sample]:
        with self.lock:
            if not self.samples:
                return []
            latest = self.samples[-1]
            start_s = latest.timestamp_s - WINDOW_SECONDS
            display = [sample for sample in self.display_samples if sample.timestamp_s >= start_s]
            if not display or display[-1].seq != latest.seq:
                display.append(latest)
        return display

    def since_snapshot(self, seq: int | None) -> list[Sample]:
        with self.lock:
            if not self.samples:
                return []
            if seq is None:
                return list(self.samples)
            values = []
            previous = None
            for sample in reversed(self.samples):
                if sample.seq <= seq:
                    previous = sample
                    break
                values.append(sample)
        values.reverse()
        if previous is not None:
            values.insert(0, previous)
        return values

    def range_snapshot(self, start_seq: int, end_seq: int) -> list[Sample]:
        with self.lock:
            values = []
            for sample in reversed(self.samples):
                if sample.seq < start_seq:
                    break
                if sample.seq <= end_seq:
                    values.append(sample)
        values.reverse()
        return values

    def recent_snapshot(self, seconds: float) -> list[Sample]:
        with self.lock:
            if not self.samples:
                return []
            start_s = self.samples[-1].timestamp_s - seconds
            values = []
            for sample in reversed(self.samples):
                if sample.timestamp_s < start_s:
                    break
                values.append(sample)
        values.reverse()
        return values

    def latest_seq(self) -> int | None:
        with self.lock:
            if not self.samples:
                return None
            return self.samples[-1].seq

    def latest(self) -> Sample | None:
        with self.lock:
            if not self.samples:
                return None
            return self.samples[-1]

    def clear(self) -> None:
        with self.lock:
            self.samples.clear()
            self.display_samples.clear()


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
        self.desired_gain_code = DEFAULT_GAIN_CODE
        self.connected = False
        self.port_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.trigger_command: str | None = None

    def reset_stream(self) -> None:
        self.samples.clear()
        self.buffer.clear()
        self.last_seq = None
        self.missing = 0

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
        self._send_gain_command()
        self._send_trigger_command()
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
        with self.write_lock:
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
            if chunk:
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
                        self.missing = 0
                    elif seq > expected:
                        self.missing += seq - expected
                for offset in range(SAMPLES_PER_BLOCK):
                    sample_seq = seq + offset
                    start = offset * RAW_BYTES
                    raw = tuple(raw24(raw_block[start:start + RAW_BYTES], index) for index in range(CHANNELS))
                    self.samples.append(
                        Sample(
                            timestamp_s=sample_seq / SAMPLE_RATE_HZ,
                            seq=sample_seq,
                            gain_code=gain_code,
                            warn_flags=warn_flags,
                            clip_flags=clip_flags,
                            raw=raw,
                            uv=raw_to_uv(raw, gain_code),
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
        self.desired_gain_code = gain_code
        if self.ser is None:
            self.messages.put("gain command needs connected board")
            return False
        return self._send_gain_command()

    def write_command(self, command: str) -> bool:
        failed = False
        with self.write_lock:
            if self.ser is None:
                self.messages.put("command needs connected board")
                return False
            try:
                self.ser.write(f"{command}\n".encode("ascii"))
            except (serial.SerialException, OSError) as exc:
                self.messages.put(f"[serial error] {exc}")
                failed = True
        if failed:
            self.disconnect()
            return False
        return True

    def _send_trigger_command(self) -> None:
        if self.trigger_command is not None and self.ser is not None:
            self.write_command(self.trigger_command)

    def _send_gain_command(self) -> bool:
        if self.ser is None:
            return False
        return self.write_command(f"gain {self.desired_gain_code}")

    def set_trigger_command(self, command: str) -> None:
        self.trigger_command = command
        self._send_trigger_command()


class PlotWindow:
    def __init__(self, reader: SerialReader, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.reader = reader
        self.samples = samples
        self.messages = messages
        self.last_message = "connecting"
        self.paused = False
        self.paused_samples: list[Sample] | None = None
        self.zero_uv = 0.0
        self.auto_zero_pending = True
        self.trigger_trace: TriggerTrace | None = None
        self.pending_trigger: PendingTrigger | None = None
        self.trigger_range_dirty = True
        self.trigger_draw_dirty = True
        self.trigger_last_seq: int | None = None
        self.trigger_checked_seq: int | None = None
        self.trigger_prev_value: float | None = None
        self.plot_start_s = 0.0
        self.plot_y_min = -1000.0
        self.plot_y_max = 1000.0
        self.fir_key: tuple[float, float] | None = None
        self.fir_values = np.array([], dtype=float)
        self.settings = QtCore.QSettings("mindaq", "capture")

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq ADC voltage live")
        self.win.resize(1250, 760)

        layout = QtWidgets.QVBoxLayout(self.win)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(6)
        self.status = QtWidgets.QLabel("port=searching | waiting for telemetry")
        self.status.setWordWrap(False)
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.status.setFixedHeight(22)
        layout.addWidget(self.status)

        button_row = QtWidgets.QHBoxLayout()
        button_row.setSpacing(8)
        self.port_box = QtWidgets.QComboBox()
        self.port_box.currentIndexChanged.connect(self.change_port)
        button_row.addWidget(self.port_box)

        self.refresh_ports_button = QtWidgets.QPushButton("Refresh Ports")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        button_row.addWidget(self.refresh_ports_button)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        button_row.addWidget(self.pause_button)

        self.zero_button = QtWidgets.QPushButton("Zero")
        self.zero_button.clicked.connect(self.zero)
        button_row.addWidget(self.zero_button)

        button_row.addWidget(QtWidgets.QLabel("ADC +"))
        self.pos_box = QtWidgets.QComboBox()
        self.fill_channel_box(self.pos_box, DEFAULT_POS_CHANNEL)
        self.pos_box.currentIndexChanged.connect(self.change_channels)
        button_row.addWidget(self.pos_box)

        button_row.addWidget(QtWidgets.QLabel("ADC -"))
        self.neg_box = QtWidgets.QComboBox()
        self.fill_channel_box(self.neg_box, DEFAULT_NEG_CHANNEL)
        self.neg_box.currentIndexChanged.connect(self.change_channels)
        button_row.addWidget(self.neg_box)

        button_row.addWidget(QtWidgets.QLabel("Gain"))
        self.gain_box = QtWidgets.QComboBox()
        for code, label in enumerate(GAIN_LABELS):
            self.gain_box.addItem(label, code)
        self.gain_box.setCurrentIndex(DEFAULT_GAIN_CODE)
        self.gain_box.currentIndexChanged.connect(self.change_gain)
        button_row.addWidget(self.gain_box)

        self.fir_enable = QtWidgets.QCheckBox("FIR")
        self.fir_enable.stateChanged.connect(self.change_fir_filter)
        button_row.addWidget(self.fir_enable)

        button_row.addWidget(QtWidgets.QLabel("Cutoff (Hz)"))
        self.fir_cutoff = QtWidgets.QDoubleSpinBox()
        self.fir_cutoff.setRange(1.0, MAX_FIR_CUTOFF_HZ)
        self.fir_cutoff.setDecimals(1)
        self.fir_cutoff.setSingleStep(10.0)
        self.fir_cutoff.setValue(DEFAULT_FIR_CUTOFF_HZ)
        self.fir_cutoff.setEnabled(False)
        self.fir_cutoff.valueChanged.connect(self.change_fir_filter)
        button_row.addWidget(self.fir_cutoff)

        button_row.addStretch(1)
        layout.addLayout(button_row)

        trigger_row = QtWidgets.QHBoxLayout()
        trigger_row.setSpacing(8)
        self.trigger_enable = QtWidgets.QCheckBox("Trigger")
        self.trigger_enable.setChecked(True)
        self.trigger_enable.stateChanged.connect(self.toggle_trigger)
        trigger_row.addWidget(self.trigger_enable)

        trigger_row.addWidget(QtWidgets.QLabel("Level (uV)"))
        self.trigger_level = QtWidgets.QDoubleSpinBox()
        self.trigger_level.setRange(-2400000.0, 2400000.0)
        self.trigger_level.setDecimals(0)
        self.trigger_level.setSingleStep(10.0)
        self.trigger_level.setValue(DEFAULT_TRIGGER_LEVEL_UV)
        self.trigger_level.valueChanged.connect(self.level_spin_changed)
        trigger_row.addWidget(self.trigger_level)

        trigger_row.addWidget(QtWidgets.QLabel("Delay (ms)"))
        self.trigger_delay = QtWidgets.QDoubleSpinBox()
        self.trigger_delay.setRange(0.0, 60000.0)
        self.trigger_delay.setDecimals(2)
        self.trigger_delay.setSingleStep(1.0)
        self.trigger_delay.setValue(DEFAULT_TRIGGER_DELAY_MS)
        self.trigger_delay.valueChanged.connect(self.change_trigger)
        trigger_row.addWidget(self.trigger_delay)

        trigger_row.addWidget(QtWidgets.QLabel("Capture window (ms)"))
        self.trigger_window_start = QtWidgets.QDoubleSpinBox()
        self.trigger_window_start.setRange(-5000.0, 5000.0)
        self.trigger_window_start.setDecimals(2)
        self.trigger_window_start.setSingleStep(1.0)
        self.trigger_window_start.setValue(DEFAULT_CAPTURE_WINDOW_START_MS)
        self.trigger_window_start.valueChanged.connect(self.change_trigger_window)
        trigger_row.addWidget(self.trigger_window_start)

        trigger_row.addWidget(QtWidgets.QLabel("to"))
        self.trigger_window_end = QtWidgets.QDoubleSpinBox()
        self.trigger_window_end.setRange(-5000.0, 5000.0)
        self.trigger_window_end.setDecimals(2)
        self.trigger_window_end.setSingleStep(1.0)
        self.trigger_window_end.setValue(DEFAULT_CAPTURE_WINDOW_END_MS)
        self.trigger_window_end.valueChanged.connect(self.change_trigger_window)
        trigger_row.addWidget(self.trigger_window_end)

        trigger_row.addWidget(QtWidgets.QLabel("Scale"))
        self.uv_per_n = QtWidgets.QDoubleSpinBox()
        self.uv_per_n.setRange(0.0000001, 1000000.0)
        self.uv_per_n.setDecimals(7)
        self.uv_per_n.setSingleStep(1.0)
        self.uv_per_n.setValue(DEFAULT_UV_PER_N)
        self.uv_per_n.setSuffix(" uV/N")
        self.uv_per_n.valueChanged.connect(self.change_plot_scale)
        trigger_row.addWidget(self.uv_per_n)
        trigger_row.addStretch(1)
        layout.addLayout(trigger_row)

        pulse_row = QtWidgets.QHBoxLayout()
        pulse_row.setSpacing(8)
        pulse_row.addWidget(QtWidgets.QLabel("Pulse GPIO"))
        self.trigger_pin = QtWidgets.QComboBox()
        for pin in TRIGGER_PINS:
            self.trigger_pin.addItem(str(pin), pin)
        self.trigger_pin.currentIndexChanged.connect(self.change_trigger)
        pulse_row.addWidget(self.trigger_pin)

        pulse_row.addWidget(QtWidgets.QLabel("Pulse width (ms)"))
        self.trigger_width = QtWidgets.QDoubleSpinBox()
        self.trigger_width.setRange(0.01, 60000.0)
        self.trigger_width.setDecimals(2)
        self.trigger_width.setSingleStep(1.0)
        self.trigger_width.setValue(1.0)
        self.trigger_width.valueChanged.connect(self.change_trigger)
        pulse_row.addWidget(self.trigger_width)
        pulse_row.addStretch(1)
        layout.addLayout(pulse_row)

        self.trigger_plot = pg.PlotWidget(title=f"Capture {channel_text(self.pos_channel(), self.neg_channel())}")
        self.trigger_plot.showGrid(x=True, y=True, alpha=0.25)
        self.trigger_plot.setLabel("left", "N")
        self.trigger_plot.setLabel("bottom", "Time from pulse (s)")
        self.trigger_curve = self.trigger_plot.plot(pen=pg.mkPen("#f39c12", width=2))
        self.trigger_dot = self.trigger_plot.plot(
            pen=None, symbol="o", symbolBrush="#f39c12", symbolPen="#ffffff", symbolSize=9
        )
        self.trigger_level_line = pg.InfiniteLine(
            angle=0, movable=False, pen=pg.mkPen("#c0392b", width=1, style=QtCore.Qt.DashLine)
        )
        self.pulse_line = pg.InfiniteLine(
            angle=90, movable=False, pen=pg.mkPen("#c0392b", width=1)
        )
        self.pulse_end_line = pg.InfiniteLine(
            angle=90, movable=False, pen=pg.mkPen("#8e44ad", width=1, style=QtCore.Qt.DotLine)
        )
        self.trigger_plot.addItem(self.trigger_level_line)
        self.trigger_plot.addItem(self.pulse_line)
        self.trigger_plot.addItem(self.pulse_end_line)
        self.pulse_line.hide()
        self.pulse_end_line.hide()

        self.plot = pg.PlotWidget(title=f"Live {channel_text(self.pos_channel(), self.neg_channel())}")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("left", "N")
        self.plot.setLabel("bottom", "Last 5 s")
        self.plot.getViewBox().sigYRangeChanged.connect(self.plot_y_range_changed)
        self.curve = self.plot.plot(pen=pg.mkPen("#2980b9", width=2))
        self.trigger_line = pg.InfiniteLine(
            angle=0, movable=False, pen=pg.mkPen("#c0392b", width=1, style=QtCore.Qt.DashLine)
        )
        self.plot.addItem(self.trigger_line)

        plot_row = QtWidgets.QHBoxLayout()
        plot_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        capture_widget = QtWidgets.QWidget()
        capture_layout = QtWidgets.QVBoxLayout(capture_widget)
        capture_layout.setContentsMargins(0, 0, 0, 0)
        capture_layout.setSpacing(4)
        capture_header = QtWidgets.QHBoxLayout()
        capture_header.setContentsMargins(0, 0, 0, 0)
        self.save_trigger_button = QtWidgets.QPushButton("Save Capture")
        self.save_trigger_button.setFixedSize(122, 24)
        self.save_trigger_button.setStyleSheet("QPushButton { background-color: #f39c12; color: white; padding: 1px 8px; }")
        self.save_trigger_button.clicked.connect(self.save_trigger_csv)
        capture_header.addWidget(self.save_trigger_button)
        capture_header.addStretch(1)
        capture_layout.addLayout(capture_header)
        capture_layout.addWidget(self.trigger_plot, 1)
        plot_splitter.addWidget(capture_widget)

        live_widget = QtWidgets.QWidget()
        live_outer_layout = QtWidgets.QVBoxLayout(live_widget)
        live_outer_layout.setContentsMargins(0, 0, 0, 0)
        live_outer_layout.setSpacing(4)
        live_header = QtWidgets.QHBoxLayout()
        live_header.setContentsMargins(0, 0, 0, 0)
        self.save_live_button = QtWidgets.QPushButton("Save Live")
        self.save_live_button.setFixedSize(96, 24)
        self.save_live_button.setStyleSheet("QPushButton { background-color: #2980b9; color: white; padding: 1px 8px; }")
        self.save_live_button.clicked.connect(self.save_live_csv)
        live_header.addWidget(self.save_live_button)
        live_header.addStretch(1)
        live_outer_layout.addLayout(live_header)
        live_layout = QtWidgets.QHBoxLayout()
        live_layout.setContentsMargins(0, 0, 0, 0)
        live_layout.addWidget(self.plot, 1)
        self.trigger_slider = QtWidgets.QSlider(QtCore.Qt.Vertical)
        self.trigger_slider.setRange(0, TRIGGER_SLIDER_STEPS)
        self.trigger_slider.setValue(TRIGGER_SLIDER_STEPS // 2)
        self.trigger_slider.setTickPosition(QtWidgets.QSlider.TicksRight)
        self.trigger_slider.setTickInterval(TRIGGER_SLIDER_STEPS // 10)
        self.trigger_slider.valueChanged.connect(self.level_slider_changed)
        live_layout.addWidget(self.trigger_slider)
        live_outer_layout.addLayout(live_layout, 1)

        plot_splitter.addWidget(live_widget)
        plot_splitter.setStretchFactor(0, 1)
        plot_splitter.setStretchFactor(1, 1)
        plot_row.addWidget(plot_splitter, 1)
        layout.addLayout(plot_row, 1)

        self.win.show()
        self.send_trigger_config()
        self.reset_trigger_trace(arm_now=True)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.refresh_ports()

    def fill_channel_box(self, box: QtWidgets.QComboBox, default: int | None) -> None:
        box.addItem("None", None)
        for channel, name in enumerate(CHANNEL_NAMES):
            box.addItem(name, channel)
        index = box.findData(default)
        box.setCurrentIndex(index if index >= 0 else 0)

    def pos_channel(self) -> int | None:
        value = self.pos_box.currentData()
        return None if value is None else int(value)

    def neg_channel(self) -> int | None:
        value = self.neg_box.currentData()
        return None if value is None else int(value)

    def sample_value(self, sample: Sample) -> float:
        return channel_value(sample.uv, self.pos_channel(), self.neg_channel())

    def selected_values(self, samples: list[Sample]) -> np.ndarray:
        pos = self.pos_channel()
        neg = self.neg_channel()
        if pos is None and neg is None:
            return np.zeros(len(samples), dtype=float)
        if pos is None:
            return np.fromiter((sample.uv[neg] for sample in samples), dtype=float, count=len(samples))
        if neg is None:
            return np.fromiter((sample.uv[pos] for sample in samples), dtype=float, count=len(samples))
        return np.fromiter((sample.uv[pos] - sample.uv[neg] for sample in samples), dtype=float, count=len(samples))

    def live_samples(self, samples: list[Sample]) -> list[Sample]:
        if not samples:
            return []
        start_s = samples[-1].timestamp_s - WINDOW_SECONDS
        values = [sample for sample in samples if sample.timestamp_s >= start_s]
        display = values[::DISPLAY_DECIMATE]
        if values and (not display or display[-1].seq != values[-1].seq):
            display.append(values[-1])
        return display

    def display_values(self, samples: list[Sample]) -> np.ndarray:
        values = self.selected_values(samples)
        if not self.fir_enable.isChecked():
            return values
        key = (float(self.fir_cutoff.value()), sample_rate(samples))
        if self.fir_key != key:
            self.fir_key = key
            self.fir_values = fir_coefficients(*key)
        return apply_fir(values, self.fir_values)

    def plot_scale(self) -> float:
        return float(self.uv_per_n.value())

    def plot_threshold(self) -> float:
        return self.trigger_threshold() / self.plot_scale()

    def plot_values(self, samples: list[Sample]) -> np.ndarray:
        return (self.display_values(samples) - self.zero_uv) / self.plot_scale()

    def average_value(self, samples: list[Sample]) -> float | None:
        if not samples:
            return None
        return float(self.selected_values(samples).mean())

    def recent_average_value(self) -> float | None:
        if self.paused_samples is not None:
            if not self.paused_samples:
                return None
            start_s = self.paused_samples[-1].timestamp_s - 1.0
            return self.average_value([sample for sample in self.paused_samples if sample.timestamp_s >= start_s])
        return self.average_value(self.samples.recent_snapshot(1.0))

    def change_fir_filter(self, *_args) -> None:
        enabled = self.fir_enable.isChecked()
        self.fir_cutoff.setEnabled(enabled)
        self.fir_key = None
        self.trigger_draw_dirty = True
        if enabled:
            self.last_message = f"FIR cutoff {self.fir_cutoff.value():.1f} Hz"
        else:
            self.last_message = "FIR off"

    def trigger_value(self, sample: Sample) -> float:
        return self.sample_value(sample) - self.zero_uv

    def trigger_threshold(self) -> float:
        return float(self.trigger_level.value())

    def trigger_delay_s(self) -> float:
        return float(self.trigger_delay.value()) / 1000.0

    def trigger_width_s(self) -> float:
        return float(self.trigger_width.value()) / 1000.0

    def pulse_start_seq(self, cross_seq: float) -> float:
        return cross_seq + self.trigger_delay_s() * SAMPLE_RATE_HZ

    def pulse_end_seq(self, cross_seq: float) -> float:
        return self.pulse_start_seq(cross_seq) + self.trigger_width_s() * SAMPLE_RATE_HZ

    def trigger_window_bounds(self, cross_seq: float) -> tuple[int, int]:
        pulse_seq = self.pulse_start_seq(cross_seq)
        start_s = float(self.trigger_window_start.value()) / 1000.0
        end_s = float(self.trigger_window_end.value()) / 1000.0
        if end_s < start_s:
            start_s, end_s = end_s, start_s
        start_seq = int(math.floor(pulse_seq + start_s * SAMPLE_RATE_HZ))
        end_seq = int(math.ceil(pulse_seq + end_s * SAMPLE_RATE_HZ))
        return start_seq, end_seq

    def trigger_x_left(self) -> float:
        start_s = float(self.trigger_window_start.value()) / 1000.0
        end_s = float(self.trigger_window_end.value()) / 1000.0
        return min(start_s, end_s)

    def set_slider_for_threshold(self) -> None:
        if not hasattr(self, "trigger_slider"):
            return
        span = self.plot_y_max - self.plot_y_min
        if span <= 0.0:
            return
        frac = (self.plot_threshold() - self.plot_y_min) / span
        value = int(round(max(0.0, min(1.0, frac)) * TRIGGER_SLIDER_STEPS))
        if self.trigger_slider.value() == value:
            return
        self.trigger_slider.blockSignals(True)
        self.trigger_slider.setValue(value)
        self.trigger_slider.blockSignals(False)

    def slider_threshold(self, value: int) -> float:
        frac = value / float(TRIGGER_SLIDER_STEPS)
        return (self.plot_y_min + frac * (self.plot_y_max - self.plot_y_min)) * self.plot_scale()

    def plot_y_range_changed(self, *_args) -> None:
        y_min, y_max = self.plot.getViewBox().viewRange()[1]
        if not math.isfinite(y_min) or not math.isfinite(y_max):
            return
        if y_max <= y_min:
            y_min -= 1.0
            y_max += 1.0
        self.plot_y_min = float(y_min)
        self.plot_y_max = float(y_max)
        self.set_slider_for_threshold()

    def reset_trigger_trace(self, arm_now: bool = False) -> None:
        self.trigger_trace = None
        self.pending_trigger = None
        self.trigger_range_dirty = True
        self.trigger_draw_dirty = True
        self.trigger_last_seq = None
        self.trigger_checked_seq = None
        self.trigger_prev_value = None
        self.trigger_curve.setData([], [])
        self.trigger_dot.setData([], [])
        if not arm_now:
            return
        latest = self.samples.latest()
        if latest is not None:
            self.trigger_checked_seq = latest.seq
            self.trigger_prev_value = self.trigger_value(latest)

    def trigger_command(self) -> str:
        if not self.trigger_enable.isChecked():
            return "trigger off"
        pos = -1 if self.pos_channel() is None else self.pos_channel()
        neg = -1 if self.neg_channel() is None else self.neg_channel()
        edge = 0
        pin = int(self.trigger_pin.currentData())
        threshold_uv1000 = int(round((self.trigger_threshold() + self.zero_uv) * 1000.0))
        delay_us = int(round(float(self.trigger_delay.value()) * 1000.0))
        width_us = max(1, int(round(float(self.trigger_width.value()) * 1000.0)))
        return f"trigger {pos} {neg} {edge} {pin} {threshold_uv1000} {delay_us} {width_us}"

    def send_trigger_config(self) -> None:
        self.reader.set_trigger_command(self.trigger_command())
        self.trigger_line.setValue(self.plot_threshold())
        self.trigger_level_line.setValue(self.plot_threshold())
        self.update_pulse_line()
        self.trigger_draw_dirty = True

    def change_trigger(self) -> None:
        self.send_trigger_config()
        self.last_message = "trigger updated"

    def toggle_trigger(self) -> None:
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "trigger updated"

    def change_trigger_hard(self) -> None:
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "trigger updated"

    def change_trigger_window(self) -> None:
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.last_message = "capture window updated"

    def change_plot_scale(self) -> None:
        self.trigger_line.setValue(self.plot_threshold())
        self.trigger_level_line.setValue(self.plot_threshold())
        self.trigger_draw_dirty = True
        self.set_slider_for_threshold()
        self.last_message = "plot scale updated"

    def level_spin_changed(self) -> None:
        self.set_slider_for_threshold()
        self.change_trigger_hard()

    def level_slider_changed(self, value: int) -> None:
        threshold = self.slider_threshold(value)
        if abs(self.trigger_level.value() - threshold) > 0.0005:
            self.trigger_level.blockSignals(True)
            self.trigger_level.setValue(threshold)
            self.trigger_level.blockSignals(False)
        self.change_trigger_hard()

    def update_pulse_line(self) -> None:
        if self.trigger_trace is None or not self.trigger_trace.samples:
            self.pulse_line.hide()
            self.pulse_end_line.hide()
            return
        self.pulse_line.show()
        self.pulse_end_line.show()
        self.trigger_trace.pulse_start_seq = self.pulse_start_seq(self.trigger_trace.cross_seq)
        self.trigger_trace.pulse_end_seq = self.pulse_end_seq(self.trigger_trace.cross_seq)
        pulse_width_x = (self.trigger_trace.pulse_end_seq - self.trigger_trace.pulse_start_seq) / SAMPLE_RATE_HZ
        self.pulse_line.setValue(0.0)
        self.pulse_end_line.setValue(pulse_width_x)

    def finish_pending_trigger(self) -> None:
        if self.pending_trigger is None:
            return
        pending = self.pending_trigger
        trace_samples = self.samples.range_snapshot(pending.pre_seq, pending.post_seq)
        if not trace_samples:
            return
        first_draw = self.trigger_trace is None
        latest_seq = self.samples.latest_seq()
        done = latest_seq is not None and latest_seq >= pending.post_seq
        self.trigger_trace = TriggerTrace(
            trace_samples,
            pending.cross_seq,
            pending.cross_uv,
            pending.pulse_start_seq,
            pending.pulse_end_seq,
        )
        self.trigger_draw_dirty = True
        if first_draw or done:
            self.trigger_range_dirty = True
        if done:
            self.pending_trigger = None
            self.last_message = "capture ready"
        else:
            self.last_message = "capturing"

    def update_trigger_trace(self, samples: list[Sample]) -> None:
        if not self.trigger_enable.isChecked() or len(samples) < 2:
            self.finish_pending_trigger()
            return
        self.finish_pending_trigger()
        threshold = self.trigger_threshold()
        start = 0
        if self.trigger_checked_seq is not None:
            for index, sample in enumerate(samples):
                if sample.seq > self.trigger_checked_seq:
                    start = max(0, index - 1)
                    break
            else:
                return

        prev_value = self.trigger_prev_value
        prev_sample = samples[start - 1] if start > 0 else None
        if prev_value is None and prev_sample is not None:
            prev_value = self.trigger_value(prev_sample)

        for sample in samples[start:]:
            value = self.trigger_value(sample)
            if prev_value is None or prev_sample is None:
                prev_value = value
                prev_sample = sample
                continue

            crossed = False
            crossed = prev_value < threshold <= value

            locked_out = self.trigger_last_seq is not None and sample.seq - self.trigger_last_seq < TRIGGER_LOCKOUT_SAMPLES
            if crossed and not locked_out and self.pending_trigger is None:
                span = value - prev_value
                frac = 0.0 if span == 0.0 else (threshold - prev_value) / span
                cross_seq = prev_sample.seq + max(0.0, min(1.0, frac))
                pulse_start_seq = self.pulse_start_seq(cross_seq)
                pulse_end_seq = self.pulse_end_seq(cross_seq)
                pre_seq, post_seq = self.trigger_window_bounds(cross_seq)
                self.pending_trigger = PendingTrigger(cross_seq, threshold, pulse_start_seq, pulse_end_seq, pre_seq, post_seq)
                self.finish_pending_trigger()
                self.trigger_last_seq = sample.seq

            prev_value = value
            prev_sample = sample

        self.trigger_prev_value = prev_value
        self.trigger_checked_seq = samples[-1].seq

    def draw_trigger_trace(self) -> None:
        if not self.trigger_draw_dirty:
            return
        if self.trigger_trace is None or not self.trigger_trace.samples:
            self.trigger_curve.setData([], [])
            self.trigger_dot.setData([], [])
            self.trigger_level_line.setValue(self.plot_threshold())
            self.update_pulse_line()
            self.trigger_draw_dirty = False
            return
        pulse_start_s = self.trigger_trace.pulse_start_seq / SAMPLE_RATE_HZ
        xs = np.fromiter(
            (sample.timestamp_s - pulse_start_s for sample in self.trigger_trace.samples),
            dtype=float,
            count=len(self.trigger_trace.samples),
        )
        ys = self.plot_values(self.trigger_trace.samples)
        cross_x = (self.trigger_trace.cross_seq - self.trigger_trace.pulse_start_seq) / SAMPLE_RATE_HZ
        self.trigger_curve.setData(xs, ys)
        self.trigger_dot.setData([cross_x], [self.trigger_trace.cross_uv / self.plot_scale()])
        self.update_pulse_line()
        if self.trigger_range_dirty:
            left = self.trigger_x_left()
            right = xs[-1] if len(xs) else 0.0
            if right <= left:
                right = left + 0.001
            self.trigger_plot.setXRange(left, right, padding=0.0)
            self.trigger_range_dirty = False
        self.trigger_draw_dirty = False

    def sample_datetime(self, sample: Sample, latest_sample: Sample, wall_now_s: float) -> str:
        sample_wall_s = wall_now_s - (latest_sample.timestamp_s - sample.timestamp_s)
        return datetime.fromtimestamp(sample_wall_s).isoformat(timespec="milliseconds")

    def common_csv_values(self, sample: Sample, pos: int | None, neg: int | None) -> tuple[float, list[str | int]]:
        value = channel_value(sample.uv, pos, neg)
        values: list[str | int] = [
            f"{value:.3f}",
            sample.seq,
            pos if pos is not None else "",
            neg if neg is not None else "",
        ]
        for index, uv in enumerate(sample.uv):
            values += [f"{uv:.3f}", sample.raw[index]]
        return value, values

    def capture_csv_values(self, sample: Sample) -> list[str | int]:
        if self.trigger_trace is None:
            return ["", "", "", "", "", "", ""]
        cross_seq = self.trigger_trace.cross_seq
        pulse_start_seq = self.pulse_start_seq(cross_seq)
        pulse_end_seq = self.pulse_end_seq(cross_seq)
        cross_marker = sample.seq == int(math.floor(cross_seq + 0.5))
        pulse_start_marker = sample.seq == int(math.floor(pulse_start_seq + 0.5))
        pulse_active = pulse_start_seq <= sample.seq < pulse_end_seq
        return [
            f"{cross_seq / SAMPLE_RATE_HZ:.8f}",
            f"{pulse_start_seq / SAMPLE_RATE_HZ:.8f}",
            f"{pulse_end_seq / SAMPLE_RATE_HZ:.8f}",
            f"{self.trigger_trace.cross_uv:.3f}",
            int(cross_marker),
            int(pulse_start_marker),
            int(pulse_active),
            int(self.trigger_pin.currentData()),
            "Rising",
        ]

    def save_samples_csv(self, samples: list[Sample], label: str) -> None:
        if not samples:
            self.last_message = f"no {label} samples to save"
            return
        default_name = f"{label}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        last_dir = Path(str(self.settings.value("last_csv_dir", "")))
        start_path = str(last_dir / default_name) if last_dir.is_dir() else default_name
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.win,
            "Save CSV",
            start_path,
            "CSV Files (*.csv)",
        )
        if not path:
            return
        self.settings.setValue("last_csv_dir", str(Path(path).parent))

        pos = self.pos_channel()
        neg = self.neg_channel()
        latest_sample = samples[-1]
        wall_now_s = time.time()
        common_header = (
            ["voltage_uv", "seq", "pos_channel", "neg_channel"]
            + [name for index in range(CHANNELS) for name in (f"adc_{index}_uv", f"adc_{index}_raw")]
        )
        flag_header = ["gain_code", "warn_flags", "clip_flags"]
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if label == "capture":
                writer.writerow(
                    ["datetime", "capture_time_s", "voltage_zeroed_uv"]
                    + ["voltage_uv", "device_time_s"]
                    + [
                        "trigger_time_s",
                        "pulse_start_time_s",
                        "pulse_end_time_s",
                        "trigger_threshold_uv",
                        "trigger_cross",
                        "pulse_start",
                        "pulse_active",
                        "trigger_gpio",
                        "trigger_edge",
                    ]
                    + ["seq", "pos_channel", "neg_channel"]
                    + [name for index in range(CHANNELS) for name in (f"adc_{index}_uv", f"adc_{index}_raw")]
                    + flag_header
                )
            else:
                writer.writerow(["datetime", "voltage_zeroed_uv", "voltage_uv", "device_time_s"] + common_header[1:] + flag_header)
            for sample in samples:
                value, common_values = self.common_csv_values(sample, pos, neg)
                row = [
                    self.sample_datetime(sample, latest_sample, wall_now_s),
                    f"{value - self.zero_uv:.3f}",
                    common_values[0],
                    f"{sample.timestamp_s:.8f}",
                ]
                if label == "capture" and self.trigger_trace is not None:
                    capture_time_s = sample.timestamp_s - self.trigger_trace.pulse_start_seq / SAMPLE_RATE_HZ
                    row.insert(1, f"{capture_time_s:.8f}")
                    row += self.capture_csv_values(sample) + common_values[1:] + [sample.gain_code, sample.warn_flags, sample.clip_flags]
                else:
                    row += common_values[1:] + [sample.gain_code, sample.warn_flags, sample.clip_flags]
                writer.writerow(row)
        self.last_message = f"saved {len(samples)} {label} samples"

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
        self.reset_trigger_trace()
        self.last_message = f"port={'auto' if port_hint is None else port_hint}"

    def change_channels(self) -> None:
        self.zero_uv = 0.0
        self.auto_zero_pending = True
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        text = channel_text(self.pos_channel(), self.neg_channel())
        self.trigger_plot.setTitle(f"Capture {text}")
        self.plot.setTitle(f"Live {text}")
        self.last_message = "channels changed"

    def change_gain(self) -> None:
        gain_code = self.gain_box.currentData()
        if gain_code is None:
            return
        self.reader.set_gain_code(int(gain_code))
        self.samples.clear()
        self.paused_samples = None
        self.auto_zero_pending = False
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "gain changed; re-zero if needed"

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.paused_samples = self.samples.snapshot() if self.paused else None
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.last_message = "plot paused" if self.paused else "plot resumed"

    def zero(self) -> None:
        average = self.recent_average_value()
        if average is None:
            self.last_message = "no samples to zero"
            return
        self.zero_uv = average
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "zero set from 1s avg"

    def auto_zero(self, samples: list[Sample]) -> None:
        average = self.recent_average_value()
        self.zero_uv = average if average is not None else self.sample_value(samples[-1])
        self.auto_zero_pending = False
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "startup zero set from 1s avg"

    def save_live_csv(self) -> None:
        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        self.save_samples_csv(samples, "live")

    def save_trigger_csv(self) -> None:
        if self.trigger_trace is None:
            self.last_message = "no capture to save"
            return
        if not self.trigger_trace.samples:
            self.last_message = "capture has no samples"
            return
        self.save_samples_csv(self.trigger_trace.samples, "capture")

    def tick(self) -> None:
        while True:
            try:
                self.last_message = self.messages.get_nowait()
            except queue.Empty:
                break

        if self.paused_samples is not None:
            samples = self.live_samples(self.paused_samples)
        elif self.trigger_enable.isChecked():
            trigger_samples = self.samples.since_snapshot(self.trigger_checked_seq)
            self.update_trigger_trace(trigger_samples)
            samples = self.samples.display_snapshot()
        else:
            samples = self.samples.display_snapshot()
        port = self.reader.current_port or "searching"
        if len(samples) < MIN_DISPLAY_SAMPLES:
            self.curve.setData([], [])
            self.draw_trigger_trace()
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
        self.plot_start_s = start_s
        xs = np.fromiter((sample.timestamp_s - start_s for sample in samples), dtype=float, count=len(samples))
        ys = self.plot_values(samples)
        self.curve.setData(xs, ys)
        self.plot.setXRange(0.0, max(WINDOW_SECONDS, xs[-1] if len(xs) else WINDOW_SECONDS), padding=0.0)
        self.trigger_line.setValue(self.plot_threshold())
        self.trigger_level_line.setValue(self.plot_threshold())
        self.draw_trigger_trace()

        latest = samples[-1]
        average = self.recent_average_value()
        average_text = "n/a" if average is None else f"{average - self.zero_uv:.1f} uV"
        headroom = [0.0] * CHANNELS
        for sample in samples:
            for index, value in enumerate(sample.raw):
                headroom[index] = max(headroom[index], abs(value) * 100.0 / ADC_FULL_SCALE)
        raw_pct = "/".join(f"{value:.0f}" for value in headroom)
        self.status.setText(
            f"port={port} | gain={GAIN_LABELS[latest.gain_code]} raw%={raw_pct} | Avg (1s): {average_text} | {self.last_message}"
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
