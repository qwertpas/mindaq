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
SAMPLE_RATE_HZ = 32_000.0
DISPLAY_RATE_HZ = 1000.0
DISPLAY_DECIMATE = int(SAMPLE_RATE_HZ / DISPLAY_RATE_HZ)
MIN_DISPLAY_SAMPLES = 250
CHANNELS = 6
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
NOTCH_Q = 10.0
NOTCH_FREQS = (60.0, 120.0)
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB", "COM")
GAIN_LABELS = ("x1", "x2", "x4", "x8", "x16", "x32", "x64", "x128")
DEFAULT_GAIN_CODE = 0
ADC_FULL_SCALE = 0x7FFFFF
TRIGGER_PINS = (43, 44, 1, 2, 3, 4, 5, 6, 7)
DEFAULT_TRIGGER_PRE_SECONDS = 0.1
DEFAULT_TRIGGER_POST_SECONDS = 0.4
TRIGGER_LOCKOUT_SECONDS = 1.0
TRIGGER_LOCKOUT_SAMPLES = int(TRIGGER_LOCKOUT_SECONDS * SAMPLE_RATE_HZ)
TRIGGER_SLIDER_STEPS = 10000


def count_to_microvolt(gain_code: int) -> float:
    return (1.2e6 / float(1 << gain_code)) / 8388608.0


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
        return f"ADC {neg}"
    if neg is None:
        return f"ADC {pos}"
    return f"ADC {pos} - ADC {neg}"


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
        output = self.b0 * value + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
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
        if self.ser is None:
            self.messages.put("gain command needs connected board")
            return False
        return self.write_command(f"gain {gain_code}")

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
        self.filter_voltage = False
        self.display_filters = [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS]
        self.zero_uv = 0.0
        self.auto_zero_pending = True
        self.trigger_trace: TriggerTrace | None = None
        self.trigger_last_seq: int | None = None
        self.trigger_checked_seq: int | None = None
        self.trigger_prev_value: float | None = None
        self.plot_start_s = 0.0
        self.plot_y_min = -1000.0
        self.plot_y_max = 1000.0
        self.trigger_draw_start_x = 0.0

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq ADC voltage live")
        self.win.resize(1250, 760)

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

        self.zero_button = QtWidgets.QPushButton("Zero")
        self.zero_button.clicked.connect(self.zero)
        button_row.addWidget(self.zero_button)

        self.filter_button = QtWidgets.QPushButton("60 Hz Filter Off")
        self.filter_button.clicked.connect(self.toggle_filter)
        button_row.addWidget(self.filter_button)

        button_row.addWidget(QtWidgets.QLabel("ADC +"))
        self.pos_box = QtWidgets.QComboBox()
        self.fill_channel_box(self.pos_box, 0)
        self.pos_box.currentIndexChanged.connect(self.change_channels)
        button_row.addWidget(self.pos_box)

        button_row.addWidget(QtWidgets.QLabel("ADC -"))
        self.neg_box = QtWidgets.QComboBox()
        self.fill_channel_box(self.neg_box, 1)
        self.neg_box.currentIndexChanged.connect(self.change_channels)
        button_row.addWidget(self.neg_box)

        button_row.addWidget(QtWidgets.QLabel("Gain"))
        self.gain_box = QtWidgets.QComboBox()
        for code, label in enumerate(GAIN_LABELS):
            self.gain_box.addItem(label, code)
        self.gain_box.setCurrentIndex(DEFAULT_GAIN_CODE)
        self.gain_box.currentIndexChanged.connect(self.change_gain)
        button_row.addWidget(self.gain_box)

        self.save_button = QtWidgets.QPushButton("Save All")
        self.save_button.clicked.connect(self.save_all_csv)
        button_row.addWidget(self.save_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        trigger_row = QtWidgets.QHBoxLayout()
        self.trigger_enable = QtWidgets.QCheckBox("Trigger")
        self.trigger_enable.stateChanged.connect(self.toggle_trigger)
        trigger_row.addWidget(self.trigger_enable)

        trigger_row.addWidget(QtWidgets.QLabel("Level uV"))
        self.trigger_level = QtWidgets.QDoubleSpinBox()
        self.trigger_level.setRange(-2400000.0, 2400000.0)
        self.trigger_level.setDecimals(3)
        self.trigger_level.setSingleStep(10.0)
        self.trigger_level.valueChanged.connect(self.level_spin_changed)
        trigger_row.addWidget(self.trigger_level)

        trigger_row.addWidget(QtWidgets.QLabel("Edge"))
        self.trigger_edge = QtWidgets.QComboBox()
        self.trigger_edge.addItem("Rising", 0)
        self.trigger_edge.addItem("Falling", 1)
        self.trigger_edge.currentIndexChanged.connect(self.change_trigger_hard)
        trigger_row.addWidget(self.trigger_edge)

        trigger_row.addWidget(QtWidgets.QLabel("GPIO"))
        self.trigger_pin = QtWidgets.QComboBox()
        for pin in TRIGGER_PINS:
            self.trigger_pin.addItem(str(pin), pin)
        self.trigger_pin.currentIndexChanged.connect(self.change_trigger)
        trigger_row.addWidget(self.trigger_pin)

        trigger_row.addWidget(QtWidgets.QLabel("Delay ms"))
        self.trigger_delay = QtWidgets.QDoubleSpinBox()
        self.trigger_delay.setRange(0.0, 60000.0)
        self.trigger_delay.setDecimals(3)
        self.trigger_delay.setSingleStep(0.1)
        self.trigger_delay.valueChanged.connect(self.change_trigger)
        trigger_row.addWidget(self.trigger_delay)

        trigger_row.addWidget(QtWidgets.QLabel("Width ms"))
        self.trigger_width = QtWidgets.QDoubleSpinBox()
        self.trigger_width.setRange(0.001, 60000.0)
        self.trigger_width.setDecimals(3)
        self.trigger_width.setSingleStep(0.1)
        self.trigger_width.setValue(1.0)
        self.trigger_width.valueChanged.connect(self.change_trigger)
        trigger_row.addWidget(self.trigger_width)
        trigger_row.addStretch(1)
        layout.addLayout(trigger_row)

        window_row = QtWidgets.QHBoxLayout()
        window_row.addWidget(QtWidgets.QLabel("Trigger Window Before s"))
        self.trigger_pre = QtWidgets.QDoubleSpinBox()
        self.trigger_pre.setRange(0.0, WINDOW_SECONDS)
        self.trigger_pre.setDecimals(3)
        self.trigger_pre.setSingleStep(0.05)
        self.trigger_pre.setValue(DEFAULT_TRIGGER_PRE_SECONDS)
        self.trigger_pre.valueChanged.connect(self.change_trigger_window)
        window_row.addWidget(self.trigger_pre)

        window_row.addWidget(QtWidgets.QLabel("After s"))
        self.trigger_post = QtWidgets.QDoubleSpinBox()
        self.trigger_post.setRange(0.001, WINDOW_SECONDS)
        self.trigger_post.setDecimals(3)
        self.trigger_post.setSingleStep(0.05)
        self.trigger_post.setValue(DEFAULT_TRIGGER_POST_SECONDS)
        self.trigger_post.valueChanged.connect(self.change_trigger_window)
        window_row.addWidget(self.trigger_post)

        self.save_trigger_button = QtWidgets.QPushButton("Save Trigger Window")
        self.save_trigger_button.clicked.connect(self.save_trigger_csv)
        window_row.addWidget(self.save_trigger_button)
        window_row.addStretch(1)
        layout.addLayout(window_row)

        self.plot = pg.PlotWidget(title=channel_text(self.pos_channel(), self.neg_channel()))
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("left", "uV")
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.getViewBox().sigYRangeChanged.connect(self.plot_y_range_changed)
        self.curve = self.plot.plot(pen=pg.mkPen("#2980b9", width=2))
        self.trigger_curve = self.plot.plot(pen=pg.mkPen("#f39c12", width=2))
        self.trigger_dot = self.plot.plot(
            pen=None, symbol="o", symbolBrush="#f39c12", symbolPen="#ffffff", symbolSize=9
        )
        self.trigger_line = pg.InfiniteLine(
            angle=0, movable=False, pen=pg.mkPen("#c0392b", width=1, style=QtCore.Qt.DashLine)
        )
        self.pulse_line = pg.InfiniteLine(
            angle=90, movable=False, pen=pg.mkPen("#8e44ad", width=1, style=QtCore.Qt.DashLine)
        )
        self.pulse_end_line = pg.InfiniteLine(
            angle=90, movable=False, pen=pg.mkPen("#8e44ad", width=1, style=QtCore.Qt.DotLine)
        )
        self.plot.addItem(self.trigger_line)
        self.plot.addItem(self.pulse_line)
        self.plot.addItem(self.pulse_end_line)

        plot_row = QtWidgets.QHBoxLayout()
        plot_row.addWidget(self.plot, 1)
        self.trigger_slider = QtWidgets.QSlider(QtCore.Qt.Vertical)
        self.trigger_slider.setRange(0, TRIGGER_SLIDER_STEPS)
        self.trigger_slider.setValue(TRIGGER_SLIDER_STEPS // 2)
        self.trigger_slider.setTickPosition(QtWidgets.QSlider.TicksRight)
        self.trigger_slider.setTickInterval(TRIGGER_SLIDER_STEPS // 10)
        self.trigger_slider.valueChanged.connect(self.level_slider_changed)
        plot_row.addWidget(self.trigger_slider)
        layout.addLayout(plot_row)

        self.win.show()
        self.send_trigger_config()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.refresh_ports()

    def fill_channel_box(self, box: QtWidgets.QComboBox, default: int | None) -> None:
        box.addItem("None", None)
        for channel in range(CHANNELS):
            box.addItem(str(channel), channel)
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

    def display_values(self, samples: list[Sample]) -> list[float]:
        values = [self.sample_value(sample) for sample in samples]
        if not self.filter_voltage:
            return values
        self.display_filters = [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS]
        filtered = []
        for value in values:
            for filt in self.display_filters:
                value = filt.step(value)
            filtered.append(value)
        return filtered

    def trigger_value(self, sample: Sample) -> float:
        return self.sample_value(sample) - self.zero_uv

    def trigger_threshold(self) -> float:
        return float(self.trigger_level.value())

    def trigger_delay_s(self) -> float:
        return float(self.trigger_delay.value()) / 1000.0

    def trigger_width_s(self) -> float:
        return float(self.trigger_width.value()) / 1000.0

    def trigger_pre_s(self) -> float:
        return float(self.trigger_pre.value())

    def trigger_post_s(self) -> float:
        return float(self.trigger_post.value())

    def pulse_start_seq(self, cross_seq: float) -> float:
        return cross_seq + self.trigger_delay_s() * SAMPLE_RATE_HZ

    def pulse_end_seq(self, cross_seq: float) -> float:
        return self.pulse_start_seq(cross_seq) + self.trigger_width_s() * SAMPLE_RATE_HZ

    def set_slider_for_threshold(self) -> None:
        if not hasattr(self, "trigger_slider"):
            return
        span = self.plot_y_max - self.plot_y_min
        if span <= 0.0:
            return
        frac = (self.trigger_threshold() - self.plot_y_min) / span
        value = int(round(max(0.0, min(1.0, frac)) * TRIGGER_SLIDER_STEPS))
        if self.trigger_slider.value() == value:
            return
        self.trigger_slider.blockSignals(True)
        self.trigger_slider.setValue(value)
        self.trigger_slider.blockSignals(False)

    def slider_threshold(self, value: int) -> float:
        frac = value / float(TRIGGER_SLIDER_STEPS)
        return self.plot_y_min + frac * (self.plot_y_max - self.plot_y_min)

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
        self.trigger_last_seq = None
        self.trigger_checked_seq = None
        self.trigger_prev_value = None
        self.trigger_curve.setData([], [])
        self.trigger_dot.setData([], [])
        if not arm_now:
            return
        samples = self.samples.snapshot()
        if samples:
            latest = samples[-1]
            self.trigger_checked_seq = latest.seq
            self.trigger_prev_value = self.trigger_value(latest)

    def trigger_command(self) -> str:
        if not self.trigger_enable.isChecked():
            return "trigger off"
        pos = -1 if self.pos_channel() is None else self.pos_channel()
        neg = -1 if self.neg_channel() is None else self.neg_channel()
        edge = int(self.trigger_edge.currentData())
        pin = int(self.trigger_pin.currentData())
        threshold_uv1000 = int(round((self.trigger_threshold() + self.zero_uv) * 1000.0))
        delay_us = int(round(float(self.trigger_delay.value()) * 1000.0))
        width_us = max(1, int(round(float(self.trigger_width.value()) * 1000.0)))
        return f"trigger {pos} {neg} {edge} {pin} {threshold_uv1000} {delay_us} {width_us}"

    def send_trigger_config(self) -> None:
        self.reader.set_trigger_command(self.trigger_command())
        self.trigger_line.setValue(self.trigger_threshold())
        self.update_pulse_line()

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
        self.last_message = "trigger window updated"

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
        trace_start_s = self.trigger_trace.samples[0].timestamp_s
        pulse_x = self.trigger_draw_start_x + self.trigger_trace.pulse_start_seq / SAMPLE_RATE_HZ - trace_start_s
        pulse_end_x = self.trigger_draw_start_x + self.trigger_trace.pulse_end_seq / SAMPLE_RATE_HZ - trace_start_s
        self.pulse_line.setValue(pulse_x)
        self.pulse_end_line.setValue(pulse_end_x)

    def update_trigger_trace(self, samples: list[Sample]) -> None:
        if not self.trigger_enable.isChecked() or len(samples) < 2:
            return
        threshold = self.trigger_threshold()
        edge = int(self.trigger_edge.currentData())
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
            if edge == 0:
                crossed = prev_value < threshold <= value
            else:
                crossed = prev_value > threshold >= value

            locked_out = self.trigger_last_seq is not None and sample.seq - self.trigger_last_seq < TRIGGER_LOCKOUT_SAMPLES
            if crossed and not locked_out:
                span = value - prev_value
                frac = 0.0 if span == 0.0 else (threshold - prev_value) / span
                cross_seq = prev_sample.seq + max(0.0, min(1.0, frac))
                pulse_start_seq = self.pulse_start_seq(cross_seq)
                pulse_end_seq = self.pulse_end_seq(cross_seq)
                pre_seq = int(cross_seq - self.trigger_pre_s() * SAMPLE_RATE_HZ)
                post_seq = int(max(cross_seq + self.trigger_post_s() * SAMPLE_RATE_HZ, pulse_end_seq))
                trace_samples = list(item for item in samples if pre_seq <= item.seq <= post_seq)
                self.trigger_trace = TriggerTrace(trace_samples, cross_seq, threshold, pulse_start_seq, pulse_end_seq)
                self.trigger_last_seq = sample.seq

            prev_value = value
            prev_sample = sample

        self.trigger_prev_value = prev_value
        self.trigger_checked_seq = samples[-1].seq

    def draw_trigger_trace(self) -> None:
        if self.trigger_trace is None or not self.trigger_trace.samples:
            self.trigger_curve.setData([], [])
            self.trigger_dot.setData([], [])
            self.update_pulse_line()
            return
        trace_start_s = self.trigger_trace.samples[0].timestamp_s
        trace_end_s = self.trigger_trace.samples[-1].timestamp_s
        trace_duration_s = max(0.001, trace_end_s - trace_start_s)
        x_min, x_max = self.plot.getViewBox().viewRange()[0]
        x_span = max(0.001, x_max - x_min)
        self.trigger_draw_start_x = x_min + max(0.0, x_span - trace_duration_s) * 0.58
        xs = [self.trigger_draw_start_x + sample.timestamp_s - trace_start_s for sample in self.trigger_trace.samples]
        ys = [self.trigger_value(sample) for sample in self.trigger_trace.samples]
        cross_x = self.trigger_draw_start_x + self.trigger_trace.cross_seq / SAMPLE_RATE_HZ - trace_start_s
        self.trigger_curve.setData(xs, ys)
        self.trigger_dot.setData([cross_x], [self.trigger_trace.cross_uv])
        self.update_pulse_line()

    def trigger_csv_values(self, sample: Sample) -> list[str | int]:
        if self.trigger_trace is None:
            return ["", "", "", "", "", "", "", "", ""]
        cross_seq = self.trigger_trace.cross_seq
        pulse_start_seq = self.pulse_start_seq(cross_seq)
        pulse_end_seq = self.pulse_end_seq(cross_seq)
        cross_marker = sample.seq == int(math.floor(cross_seq + 0.5))
        pulse_start_marker = sample.seq == int(math.floor(pulse_start_seq + 0.5))
        pulse_active = pulse_start_seq <= sample.seq < pulse_end_seq
        return [
            int(cross_marker),
            int(pulse_start_marker),
            int(pulse_active),
            f"{cross_seq / SAMPLE_RATE_HZ:.8f}",
            f"{pulse_start_seq / SAMPLE_RATE_HZ:.8f}",
            f"{pulse_end_seq / SAMPLE_RATE_HZ:.8f}",
            f"{self.trigger_trace.cross_uv:.3f}",
            int(self.trigger_pin.currentData()),
            self.trigger_edge.currentText(),
        ]

    def trigger_status_text(self) -> str:
        if self.trigger_trace is None:
            return ""
        cross_s = self.trigger_trace.cross_seq / SAMPLE_RATE_HZ
        pulse_start_s = self.pulse_start_seq(self.trigger_trace.cross_seq) / SAMPLE_RATE_HZ
        pulse_end_s = self.pulse_end_seq(self.trigger_trace.cross_seq) / SAMPLE_RATE_HZ
        return f" | trigger={cross_s:.6f}s pulse={pulse_start_s:.6f}-{pulse_end_s:.6f}s"

    def save_samples_csv(self, samples: list[Sample], label: str) -> None:
        if not samples:
            self.last_message = f"no {label} samples to save"
            return
        default_name = f"mindaq_adc_voltage_{label}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self.win, "Save CSV", default_name, "CSV Files (*.csv)")
        if not path:
            return

        pos = self.pos_channel()
        neg = self.neg_channel()
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ["timestamp_s", "seq", "gain_code", "warn_flags", "clip_flags", "pos_channel", "neg_channel"]
                + [f"adc_{index}_raw" for index in range(CHANNELS)]
                + [f"adc_{index}_uv" for index in range(CHANNELS)]
                + [
                    "voltage_uv",
                    "voltage_zeroed_uv",
                    "trigger_cross",
                    "pulse_start",
                    "pulse_active",
                    "trigger_time_s",
                    "pulse_start_time_s",
                    "pulse_end_time_s",
                    "trigger_threshold_uv",
                    "trigger_gpio",
                    "trigger_edge",
                ]
            )
            for sample in samples:
                value = channel_value(sample.uv, pos, neg)
                writer.writerow(
                    [f"{sample.timestamp_s:.8f}", sample.seq, sample.gain_code, sample.warn_flags, sample.clip_flags, pos, neg]
                    + list(sample.raw)
                    + [f"{uv:.3f}" for uv in sample.uv]
                    + [f"{value:.3f}", f"{value - self.zero_uv:.3f}"]
                    + self.trigger_csv_values(sample)
                )
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
        self.display_filters = [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS]
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.plot.setTitle(channel_text(self.pos_channel(), self.neg_channel()))
        self.last_message = "channels changed"

    def change_gain(self) -> None:
        gain_code = self.gain_box.currentData()
        if gain_code is None:
            return
        self.reader.set_gain_code(int(gain_code))
        self.samples.clear()
        self.display_filters = [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS]
        self.paused_samples = None
        self.auto_zero_pending = False
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "gain changed; re-zero if needed"

    def toggle_filter(self) -> None:
        self.filter_voltage = not self.filter_voltage
        self.display_filters = [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS]
        self.filter_button.setText("60 Hz Filter On" if self.filter_voltage else "60 Hz Filter Off")
        self.last_message = "60/120 Hz notch on" if self.filter_voltage else "60/120 Hz notch off"

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.paused_samples = self.samples.snapshot() if self.paused else None
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.last_message = "plot paused" if self.paused else "plot resumed"

    def zero(self) -> None:
        samples = self.samples.snapshot()
        if not samples:
            self.last_message = "no sample to zero"
            return
        self.zero_uv = self.sample_value(samples[-1])
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "zero set"

    def auto_zero(self, samples: list[Sample]) -> None:
        self.zero_uv = self.sample_value(samples[-1])
        self.auto_zero_pending = False
        self.reset_trigger_trace(arm_now=self.trigger_enable.isChecked())
        self.send_trigger_config()
        self.last_message = "startup zero set"

    def save_all_csv(self) -> None:
        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        self.save_samples_csv(samples, "all")

    def save_trigger_csv(self) -> None:
        if self.trigger_trace is None:
            self.last_message = "no trigger window to save"
            return
        if not self.trigger_trace.samples:
            self.last_message = "trigger window has no samples"
            return
        self.save_samples_csv(self.trigger_trace.samples, "trigger_window")

    def tick(self) -> None:
        while True:
            try:
                self.last_message = self.messages.get_nowait()
            except queue.Empty:
                break

        full_samples = None
        if self.paused_samples is not None:
            full_samples = self.paused_samples
            samples = full_samples[::DISPLAY_DECIMATE]
        elif self.trigger_enable.isChecked():
            full_samples = self.samples.snapshot()
            samples = full_samples[::DISPLAY_DECIMATE]
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
        xs = [sample.timestamp_s - start_s for sample in samples]
        values = self.display_values(samples)
        ys = [value - self.zero_uv for value in values]
        self.curve.setData(xs, ys)
        if full_samples is not None:
            self.update_trigger_trace(full_samples)
        self.trigger_line.setValue(self.trigger_threshold())
        self.draw_trigger_trace()

        latest = samples[-1]
        headroom = [0.0] * CHANNELS
        for sample in samples:
            for index, value in enumerate(sample.raw):
                headroom[index] = max(headroom[index], abs(value) * 100.0 / ADC_FULL_SCALE)
        raw_pct = "/".join(f"{value:.0f}" for value in headroom)
        self.status.setText(
            f"port={port} | gain={GAIN_LABELS[latest.gain_code]} raw%={raw_pct} | {self.last_message}"
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
