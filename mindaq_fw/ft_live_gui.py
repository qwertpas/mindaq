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
RAW_CHANNELS = ["FT_G0", "FT_G1", "FT_G2", "FT_G3", "FT_G4", "FT_G5"]
FT_CHANNELS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
PACKET = struct.Struct("<HI6fB")
SYNC = 0xA55A
PACKET_SIZE = PACKET.size
WINDOW_SECONDS = 5.0
DEFAULT_RATE_HZ = 2000.0
COLORS = ["#ff6b35", "#f39c12", "#27ae60", "#2980b9", "#8e44ad", "#c0392b"]
MAX_FORWARD_JUMP_US = 250_000
WRAP_LOW_US = 10_000_000
WRAP_HIGH_US = 0xF0000000
NOTCH_Q = 10.0
NOTCH_FREQS = (60.0, 120.0)
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB")


def gages_to_ft(values: tuple[float, ...]) -> tuple[float, ...]:
    g0, g1, g2, g3, g4, g5 = values
    fx = 0.5 * (g1 - g3)
    fy = 0.25 * (-g1 - g3 + 2.0 * g5)
    fz = 0.25 * (g0 + g2 + 2.0 * g4)
    tx = 0.5 * (g2 - g0)
    ty = 0.25 * (g0 + g2 - 2.0 * g4)
    tz = 0.25 * (g1 + g3 + 2.0 * g5)
    return (fx, fy, fz, tx, ty, tz)


def plot_position(index: int, show_ft: bool) -> tuple[int, int]:
    if show_ft:
        return index % 3, index // 3
    return index // 2, index % 2


def find_serial_port(pattern: str | None) -> str | None:
    if pattern:
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


@dataclass
class Sample:
    timestamp_s: float
    ft_uv: tuple[float, ...]
    ft_filtered: tuple[float, ...]


class SampleBuffer:
    def __init__(self, seconds: float = WINDOW_SECONDS, rate_hz: float = DEFAULT_RATE_HZ) -> None:
        self.samples: deque[Sample] = deque(maxlen=max(1000, int(seconds * rate_hz)))
        self.lock = threading.Lock()

    def append(self, sample: Sample) -> None:
        with self.lock:
            self.samples.append(sample)

    def snapshot(self) -> list[Sample]:
        with self.lock:
            return list(self.samples)

    def clear(self) -> None:
        with self.lock:
            self.samples.clear()


class NotchFilter:
    def __init__(self, freq_hz: float, sample_rate_hz: float = DEFAULT_RATE_HZ, q: float = NOTCH_Q) -> None:
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
        self.wrap_offset_us = 0
        self.last_timestamp_us: int | None = None
        self.ft_filters = [
            [NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS
        ]
        self.connected = False

    def reset_stream(self) -> None:
        self.samples.clear()
        self.buffer.clear()
        self.wrap_offset_us = 0
        self.last_timestamp_us = None
        self.ft_filters = [[NotchFilter(freq_hz) for freq_hz in NOTCH_FREQS] for _ in FT_CHANNELS]

    def connect(self) -> bool:
        port = find_serial_port(self.port_hint)
        if port is None:
            if self.connected:
                self.messages.put("searching for board")
            self.connected = False
            self.current_port = self.port_hint
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
        self.reset_stream()
        self.messages.put(f"connected {port}")
        return True

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
                chunk = self.ser.read(4096)
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
        while len(self.buffer) >= PACKET_SIZE:
            if self.buffer[0] != 0x5A or self.buffer[1] != 0xA5:
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[:PACKET_SIZE])
            checksum = 0
            for value in frame[:-1]:
                checksum ^= value
            if checksum != frame[-1]:
                del self.buffer[0]
                continue

            sync, timestamp_us, *values, _ = PACKET.unpack(frame)
            if sync == SYNC:
                if self.last_timestamp_us is not None:
                    if timestamp_us < self.last_timestamp_us:
                        if self.last_timestamp_us >= WRAP_HIGH_US and timestamp_us <= WRAP_LOW_US:
                            self.wrap_offset_us += 1 << 32
                        else:
                            del self.buffer[0]
                            continue
                    elif timestamp_us - self.last_timestamp_us > MAX_FORWARD_JUMP_US:
                        del self.buffer[0]
                        continue
                self.last_timestamp_us = timestamp_us
                full_timestamp_s = (self.wrap_offset_us + timestamp_us) / 1e6
                ft_filtered = list(gages_to_ft(tuple(values)))
                for index, value in enumerate(ft_filtered):
                    for filt in self.ft_filters[index]:
                        value = filt.step(value)
                    ft_filtered[index] = value
                self.samples.append(
                    Sample(
                        timestamp_s=full_timestamp_s,
                        ft_uv=tuple(values),
                        ft_filtered=tuple(ft_filtered),
                    )
                )

            del self.buffer[:PACKET_SIZE]

    def stop(self) -> None:
        self.stop_event.set()
        self.disconnect()


class PlotWindow:
    def __init__(self, reader: SerialReader, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.reader = reader
        self.samples = samples
        self.messages = messages
        self.last_message = "connecting"
        self.paused = False
        self.paused_samples: list[Sample] | None = None
        self.show_ft = True
        self.filter_ft = True
        self.zero_raw = [0.0] * len(RAW_CHANNELS)
        self.zero_ft = [0.0] * len(FT_CHANNELS)
        self.auto_zero_pending = True

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq force/torque live")
        self.win.resize(1400, 900)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel("port=searching | waiting for telemetry")
        layout.addWidget(self.status)

        button_row = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        button_row.addWidget(self.pause_button)

        self.zero_button = QtWidgets.QPushButton("Zero All")
        self.zero_button.clicked.connect(self.zero_all)
        button_row.addWidget(self.zero_button)

        self.mode_button = QtWidgets.QPushButton("Show Voltages")
        self.mode_button.clicked.connect(self.toggle_mode)
        button_row.addWidget(self.mode_button)

        self.filter_button = QtWidgets.QPushButton("60 Hz Filter On")
        self.filter_button.clicked.connect(self.toggle_filter)
        button_row.addWidget(self.filter_button)

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
            plot.setLabel("left", "arb")
            plot.setLabel("bottom", "Time (s)")
            curve = plot.plot(pen=pg.mkPen(COLORS[index], width=2))
            self.plots.append(plot)
            self.curves.append(curve)
            row, col = plot_position(index, True)
            self.grid.addWidget(plot, row, col)

        self.win.show()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    def current_names(self) -> list[str]:
        return FT_CHANNELS if self.show_ft else RAW_CHANNELS

    def current_unit(self) -> str:
        return "arb" if self.show_ft else "uV"

    def sample_values(self, sample: Sample) -> tuple[float, ...]:
        if not self.show_ft:
            return sample.ft_uv
        return sample.ft_filtered if self.filter_ft else gages_to_ft(sample.ft_uv)

    def zero_values(self) -> list[float]:
        return self.zero_ft if self.show_ft else self.zero_raw

    def toggle_mode(self) -> None:
        self.show_ft = not self.show_ft
        self.mode_button.setText("Show Voltages" if self.show_ft else "Show Unscaled F/T")
        self.last_message = "showing unscaled F/T" if self.show_ft else "showing voltages"
        unit = self.current_unit()
        for index, (plot, name) in enumerate(zip(self.plots, self.current_names())):
            plot.setTitle(name)
            plot.setLabel("left", unit)
            self.grid.removeWidget(plot)
            row, col = plot_position(index, self.show_ft)
            self.grid.addWidget(plot, row, col)

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
        self.zero_raw = list(latest.ft_uv)
        self.zero_ft = list(self.sample_values(latest) if self.show_ft else gages_to_ft(latest.ft_uv))
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
        header.extend(f"{name}_uv_raw" for name in RAW_CHANNELS)
        header.extend(f"{name}_uv_zeroed" for name in RAW_CHANNELS)
        header.extend(f"{name}_raw" for name in FT_CHANNELS)
        header.extend(f"{name}_zeroed" for name in FT_CHANNELS)
        header.extend(f"{name}_filtered" for name in FT_CHANNELS)
        header.extend(f"{name}_filtered_zeroed" for name in FT_CHANNELS)

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for sample in samples:
                ft_values = gages_to_ft(sample.ft_uv)
                writer.writerow(
                    [f"{sample.timestamp_s:.6f}"]
                    + [f"{value:.3f}" for value in sample.ft_uv]
                    + [f"{value - zero:.3f}" for value, zero in zip(sample.ft_uv, self.zero_raw)]
                    + [f"{value:.3f}" for value in ft_values]
                    + [f"{value - zero:.3f}" for value, zero in zip(ft_values, self.zero_ft)]
                    + [f"{value:.3f}" for value in sample.ft_filtered]
                    + [f"{value - zero:.3f}" for value, zero in zip(sample.ft_filtered, self.zero_ft)]
                )

        self.last_message = f"saved {len(samples)} samples"

    def tick(self) -> None:
        while True:
            try:
                self.last_message = self.messages.get_nowait()
            except queue.Empty:
                break

        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        port = self.reader.current_port or "searching"
        if not samples:
            for curve in self.curves:
                curve.setData([], [])
            self.auto_zero_pending = True
            self.status.setText(f"port={port} | {self.last_message}")
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

        for index, curve in enumerate(self.curves):
            ys = [self.sample_values(sample)[index] - zero[index] for sample in samples]
            curve.setData(xs, ys)

        rate_hz = 0.0
        if len(samples) > 8:
            window = samples[-256:] if len(samples) > 256 else samples
            dt = window[-1].timestamp_s - window[0].timestamp_s
            if dt > 0:
                rate_hz = (len(window) - 1) / dt

        latest_values = self.sample_values(latest)
        latest_text = " ".join(
            f"{name}={value - offset:+.1f}{self.current_unit()}"
            for name, value, offset in zip(self.current_names(), latest_values, zero)
        )
        self.status.setText(
            f"port={port} | rate={rate_hz:.1f} Hz | {latest_text} | {self.last_message}"
        )

    def exec(self) -> int:
        return self.app.exec()


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
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
