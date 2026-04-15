#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import csv
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


def find_serial_port(pattern: str | None) -> str:
    if pattern:
        matches = sorted(glob(pattern))
        return matches[0] if matches else pattern

    if list_ports is not None:
        ports = list(list_ports.comports())

        def score(port_info) -> tuple[int, str]:
            text = " ".join(
                [
                    port_info.device or "",
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            value = 0
            if getattr(port_info, "vid", None) == 0x303A:
                value += 100
            if "esp32" in text or "espressif" in text:
                value += 80
            if "usbmodem" in text or "cdc" in text:
                value += 20
            return value, port_info.device or ""

        if ports:
            return sorted(ports, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for candidate in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(candidate))
        if matches:
            return matches[0]

    raise SystemExit("No serial port found")


@dataclass
class Sample:
    timestamp_s: float
    ft_uv: tuple[float, ...]


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


class SerialReader(threading.Thread):
    def __init__(self, ser: serial.Serial, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.samples = samples
        self.messages = messages
        self.stop_event = threading.Event()
        self.buffer = bytearray()
        self.wrap_offset_us = 0
        self.last_timestamp_us: int | None = None

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError) as exc:
                self.messages.put(f"[serial error] {exc}")
                return
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
                if self.last_timestamp_us is not None and timestamp_us < self.last_timestamp_us:
                    self.wrap_offset_us += 1 << 32
                self.last_timestamp_us = timestamp_us
                full_timestamp_s = (self.wrap_offset_us + timestamp_us) / 1e6
                self.samples.append(Sample(timestamp_s=full_timestamp_s, ft_uv=tuple(values)))

            del self.buffer[:PACKET_SIZE]

    def stop(self) -> None:
        self.stop_event.set()


class PlotWindow:
    def __init__(self, port: str, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.port = port
        self.samples = samples
        self.messages = messages
        self.last_message = "connecting"
        self.paused = False
        self.paused_samples: list[Sample] | None = None
        self.show_ft = True
        self.zero_raw = [0.0] * len(RAW_CHANNELS)
        self.zero_ft = [0.0] * len(FT_CHANNELS)

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq force/torque live")
        self.win.resize(1400, 900)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel(f"port={port} | waiting for telemetry")
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
        return gages_to_ft(sample.ft_uv) if self.show_ft else sample.ft_uv

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
                )

        self.last_message = f"saved {len(samples)} samples"

    def tick(self) -> None:
        while True:
            try:
                self.last_message = self.messages.get_nowait()
            except queue.Empty:
                break

        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        if not samples:
            self.status.setText(f"port={self.port} | {self.last_message}")
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
            f"port={self.port} | rate={rate_hz:.1f} Hz | {latest_text} | {self.last_message}"
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

    port = find_serial_port(args.port)
    ser = open_serial(port)
    samples = SampleBuffer()
    messages: queue.Queue[str] = queue.Queue()
    reader = SerialReader(ser, samples, messages)
    reader.start()

    window = PlotWindow(port, samples, messages)
    try:
        return window.exec()
    finally:
        reader.stop()
        reader.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
