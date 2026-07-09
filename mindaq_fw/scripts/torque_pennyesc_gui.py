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
from pathlib import Path

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
SYNC = 0xA55A
SYNC_BYTES = b"\x5a\xa5"
PACKET = struct.Struct("<HIIffhBHhHHHHIIIIIIHBBBB")
PACKET_SIZE = PACKET.size
SAMPLE_BUFFER_SECONDS = 10.0
DISPLAY_SECONDS = 10.0
ZERO_SECONDS = 1.0
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB", "COM")
ADC_CHANNEL_NAMES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")


@dataclass
class Sample:
    timestamp_s: float
    seq: int
    torque_z_nm: float
    motor_speed_rad_s: float
    set_duty: int
    motor_id: int
    scan_mask: int
    penny_clip: int
    penny_mct_fault_count: int
    penny_isr_us: int
    penny_isr_max_us: int
    penny_i2c_us: int
    penny_isr_overrun_count: int
    penny_i2c_timeout_count: int
    penny_i2c_nack_count: int
    penny_i2c_recover_count: int
    penny_uart_overrun_errors: int
    penny_tmag_sample_count: int
    penny_tmag_sample_dt_us: int
    gain_code: int
    warn_flags: int
    clip_flags: int


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.wrap_offset_us = 0
        self.last_timestamp_us: int | None = None

    def append(self, data: bytes) -> list[Sample]:
        self.buffer.extend(data)
        samples: list[Sample] = []
        while len(self.buffer) >= PACKET_SIZE:
            if self.buffer[:2] != SYNC_BYTES:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:PACKET_SIZE])
            checksum = 0
            for value in frame[:-1]:
                checksum ^= value
            if checksum != frame[-1]:
                del self.buffer[0]
                continue

            (
                sync,
                seq,
                timestamp_us,
                torque_z_nm,
                motor_speed_rad_s,
                set_duty,
                motor_id,
                scan_mask,
                penny_clip,
                penny_mct_fault_count,
                penny_isr_us,
                penny_isr_max_us,
                penny_i2c_us,
                penny_isr_overrun_count,
                penny_i2c_timeout_count,
                penny_i2c_nack_count,
                penny_i2c_recover_count,
                penny_uart_overrun_errors,
                penny_tmag_sample_count,
                penny_tmag_sample_dt_us,
                gain_code,
                warn_flags,
                clip_flags,
                _,
            ) = PACKET.unpack(frame)
            if sync == SYNC:
                if self.last_timestamp_us is not None and timestamp_us < self.last_timestamp_us:
                    self.wrap_offset_us += 1 << 32
                self.last_timestamp_us = timestamp_us
                samples.append(
                    Sample(
                        timestamp_s=(self.wrap_offset_us + timestamp_us) / 1e6,
                        seq=seq,
                        torque_z_nm=torque_z_nm,
                        motor_speed_rad_s=motor_speed_rad_s,
                        set_duty=set_duty,
                        motor_id=motor_id,
                        scan_mask=scan_mask,
                        penny_clip=penny_clip,
                        penny_mct_fault_count=penny_mct_fault_count,
                        penny_isr_us=penny_isr_us,
                        penny_isr_max_us=penny_isr_max_us,
                        penny_i2c_us=penny_i2c_us,
                        penny_isr_overrun_count=penny_isr_overrun_count,
                        penny_i2c_timeout_count=penny_i2c_timeout_count,
                        penny_i2c_nack_count=penny_i2c_nack_count,
                        penny_i2c_recover_count=penny_i2c_recover_count,
                        penny_uart_overrun_errors=penny_uart_overrun_errors,
                        penny_tmag_sample_count=penny_tmag_sample_count,
                        penny_tmag_sample_dt_us=penny_tmag_sample_dt_us,
                        gain_code=gain_code,
                        warn_flags=warn_flags,
                        clip_flags=clip_flags,
                    )
                )
            del self.buffer[:PACKET_SIZE]
        return samples


class SampleBuffer:
    def __init__(self, seconds: float = SAMPLE_BUFFER_SECONDS, rate_hz: float = 1000.0) -> None:
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
        self.parser = FrameParser()

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError) as exc:
                self.messages.put(f"[serial error] {exc}")
                return
            if not chunk:
                continue
            for sample in self.parser.append(chunk):
                self.samples.append(sample)

    def stop(self) -> None:
        self.stop_event.set()


class PlotWindow:
    def __init__(self, ser: serial.Serial, port: str, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.ser = ser
        self.port = port
        self.samples = samples
        self.messages = messages
        self.write_lock = threading.Lock()
        self.last_message = "waiting for telemetry"
        self.paused = False
        self.paused_samples: list[Sample] | None = None
        self.torque_zero_nm = 0.0
        self.last_scan_mask: int | None = None
        self.pending_clip: int | None = None
        self.pending_clip_until = 0.0
        self.pending_gain: int | None = None
        self.pending_gain_until = 0.0
        self.settings = QtCore.QSettings("mindaq", "torque_pennyesc")

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq torque PennyESC")
        self.win.resize(1100, 760)

        layout = QtWidgets.QVBoxLayout(self.win)
        self.status = QtWidgets.QLabel(f"port={port} | waiting for telemetry")
        layout.addWidget(self.status)

        motor_row = QtWidgets.QHBoxLayout()
        self.scan_button = QtWidgets.QPushButton("Scan")
        self.scan_button.clicked.connect(self.scan_motor_ids)
        motor_row.addWidget(self.scan_button)

        motor_row.addWidget(QtWidgets.QLabel("Motor"))
        self.motor_combo = QtWidgets.QComboBox()
        self.motor_combo.addItems([str(id) for id in range(16)])
        self.motor_combo.setCurrentText("1")
        self.motor_combo.currentTextChanged.connect(self.select_motor_id)
        motor_row.addWidget(self.motor_combo)

        self.setpoint = QtWidgets.QSpinBox()
        self.setpoint.setRange(-799, 799)
        self.setpoint.setSingleStep(10)
        self.setpoint.lineEdit().returnPressed.connect(self.send_setpoint)
        motor_row.addWidget(self.setpoint)
        motor_row.addWidget(QtWidgets.QLabel("duty"))

        self.set_button = QtWidgets.QPushButton("Set")
        self.set_button.clicked.connect(self.send_setpoint)
        motor_row.addWidget(self.set_button)

        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_motor)
        motor_row.addWidget(self.stop_button)

        motor_row.addWidget(QtWidgets.QLabel("Clip"))
        self.clip = QtWidgets.QSpinBox()
        self.clip.setRange(0, 799)
        self.clip.setValue(150)
        self.clip.editingFinished.connect(self.send_clip)
        motor_row.addWidget(self.clip)

        self.clip_button = QtWidgets.QPushButton("Set Clip")
        self.clip_button.clicked.connect(self.send_clip)
        motor_row.addWidget(self.clip_button)

        motor_row.addWidget(QtWidgets.QLabel("Gain"))
        self.gain = QtWidgets.QSpinBox()
        self.gain.setRange(0, 7)
        self.gain.setValue(3)
        self.gain.editingFinished.connect(self.send_gain)
        motor_row.addWidget(self.gain)

        self.gain_button = QtWidgets.QPushButton("Set Gain")
        self.gain_button.clicked.connect(self.send_gain)
        motor_row.addWidget(self.gain_button)
        motor_row.addStretch(1)
        layout.addLayout(motor_row)

        button_row = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        button_row.addWidget(self.pause_button)

        self.save_button = QtWidgets.QPushButton("Save CSV")
        self.save_button.clicked.connect(self.save_csv)
        button_row.addWidget(self.save_button)

        self.zero_torque_button = QtWidgets.QPushButton("Zero Torque")
        self.zero_torque_button.clicked.connect(self.zero_torque)
        button_row.addWidget(self.zero_torque_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self.speed_plot = pg.PlotWidget(title="Motor Speed")
        self.torque_plot = pg.PlotWidget(title="Torque Z")
        self.duty_plot = pg.PlotWidget(title="Duty")
        for plot, label in (
            (self.speed_plot, "Speed (rad/s)"),
            (self.torque_plot, "Torque Z (Nm)"),
            (self.duty_plot, "Duty"),
        ):
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", label)
            layout.addWidget(plot)

        self.duty_plot.setLabel("bottom", "Time (s)")
        self.torque_plot.setXLink(self.speed_plot)
        self.duty_plot.setXLink(self.speed_plot)

        self.speed_curve = self.speed_plot.plot(
            pen=pg.mkPen("#f1c40f", width=2),
            symbol="o",
            symbolSize=2,
            symbolBrush="#f1c40f",
            symbolPen=None,
        )
        self.torque_curve = self.torque_plot.plot(
            pen=pg.mkPen("#ff6b35", width=2),
            symbol="o",
            symbolSize=2,
            symbolBrush="#ff6b35",
            symbolPen=None,
        )
        self.duty_curve = self.duty_plot.plot(
            pen=pg.mkPen("#2ca02c", width=2),
            symbol="o",
            symbolSize=2,
            symbolBrush="#2ca02c",
            symbolPen=None,
        )

        self.win.show()
        self.setpoint.setFocus()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def write_line(self, text: str) -> bool:
        try:
            with self.write_lock:
                self.ser.write(f"{text}\n".encode("utf-8"))
                self.ser.flush()
        except (serial.SerialException, OSError) as exc:
            self.last_message = f"[serial error] {exc}"
            self.close()
            return False
        return True

    def scan_motor_ids(self) -> None:
        if self.write_line("S"):
            self.last_message = "scanning motor IDs"

    def select_motor_id(self, text: str) -> None:
        if text and self.write_line(f"I{text}"):
            self.last_message = f"selected motor ID {text}"

    def send_setpoint(self) -> None:
        value = self.setpoint.value()
        if self.write_line(f"D{value}"):
            self.last_message = f"sent D{value}"

    def stop_motor(self) -> None:
        self.setpoint.setValue(0)
        self.send_setpoint()

    def send_clip(self) -> None:
        value = self.clip.value()
        if self.write_line(f"C{value}"):
            self.pending_clip = value
            self.pending_clip_until = time.monotonic() + 1.0
            self.last_message = f"sent C{value}"

    def send_gain(self) -> None:
        value = self.gain.value()
        if self.write_line(f"gain {value}"):
            self.pending_gain = value
            self.pending_gain_until = time.monotonic() + 1.0
            self.last_message = f"sent gain {value}"

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.paused_samples = self.samples.snapshot() if self.paused else None
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.last_message = "plot paused" if self.paused else "plot resumed"

    def zero_torque(self) -> None:
        samples = self.samples.snapshot()
        if not samples:
            self.last_message = "no torque sample to zero"
            return
        cutoff = samples[-1].timestamp_s - ZERO_SECONDS
        zero_samples = [sample.torque_z_nm for sample in samples if sample.timestamp_s >= cutoff]
        if not zero_samples:
            self.last_message = "no recent torque sample to zero"
            return
        self.torque_zero_nm = sum(zero_samples) / len(zero_samples)
        self.last_message = f"torque zero set from {len(zero_samples)} samples"

    def save_csv(self) -> None:
        samples = self.paused_samples if self.paused_samples is not None else self.samples.snapshot()
        if not samples:
            self.last_message = "no samples to save"
            return

        default_name = f"torque_pennyesc_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "timestamp_s",
                    "seq",
                    "torque_z_nm",
                    "zeroed_torque_z_nm",
                    "motor_speed_rad_s",
                    "set_duty",
                    "motor_id",
                    "clip",
                    "gain_code",
                    "warn_flags",
                    "clip_flags",
                    "penny_mct_fault_count",
                    "penny_isr_us",
                    "penny_isr_max_us",
                    "penny_i2c_us",
                    "penny_isr_overrun_count",
                    "penny_i2c_timeout_count",
                    "penny_i2c_nack_count",
                    "penny_i2c_recover_count",
                    "penny_uart_overrun_errors",
                    "penny_tmag_sample_count",
                    "penny_tmag_sample_dt_us",
                ]
            )
            for sample in samples:
                writer.writerow(
                    [
                        f"{sample.timestamp_s:.6f}",
                        str(sample.seq),
                        f"{sample.torque_z_nm:.9f}",
                        f"{self.zeroed_torque(sample):.9f}",
                        f"{sample.motor_speed_rad_s:.6f}",
                        str(sample.set_duty),
                        str(sample.motor_id),
                        str(sample.penny_clip),
                        str(sample.gain_code),
                        str(sample.warn_flags),
                        str(sample.clip_flags),
                        str(sample.penny_mct_fault_count),
                        str(sample.penny_isr_us),
                        str(sample.penny_isr_max_us),
                        str(sample.penny_i2c_us),
                        str(sample.penny_isr_overrun_count),
                        str(sample.penny_i2c_timeout_count),
                        str(sample.penny_i2c_nack_count),
                        str(sample.penny_i2c_recover_count),
                        str(sample.penny_uart_overrun_errors),
                        str(sample.penny_tmag_sample_count),
                        str(sample.penny_tmag_sample_dt_us),
                    ]
                )
        self.last_message = f"saved {len(samples)} samples"

    def zeroed_torque(self, sample: Sample) -> float:
        return sample.torque_z_nm - self.torque_zero_nm

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
        latest = samples[-1]
        self.sync_motor_controls(latest)
        self.status.setText(self.status_text(latest))
        if self.paused:
            return

        start = latest.timestamp_s - DISPLAY_SECONDS
        display_samples = [sample for sample in samples if sample.timestamp_s >= start]
        xs = [sample.timestamp_s - latest.timestamp_s for sample in display_samples]
        torque_values = [self.zeroed_torque(sample) for sample in display_samples]
        self.speed_curve.setData(xs, [sample.motor_speed_rad_s for sample in display_samples])
        self.torque_curve.setData(xs, torque_values)
        self.duty_curve.setData(xs, [sample.set_duty for sample in display_samples])
        self.torque_plot.setTitle(
            f"Torque Z | avg 5s={self.average_torque(display_samples, 5.0):.5f} Nm | "
            f"avg 1s={self.average_torque(display_samples, 1.0):.5f} Nm"
        )
        self.speed_plot.setXRange(-DISPLAY_SECONDS, 0.0, padding=0)

    def average_torque(self, samples: list[Sample], seconds: float) -> float:
        if not samples:
            return 0.0
        start = samples[-1].timestamp_s - seconds
        values = [self.zeroed_torque(sample) for sample in samples if sample.timestamp_s >= start]
        return sum(values) / len(values) if values else 0.0

    def sync_motor_controls(self, sample: Sample) -> None:
        if self.last_scan_mask != sample.scan_mask and sample.scan_mask != 0:
            self.last_scan_mask = sample.scan_mask
            found = [id for id in range(16) if (sample.scan_mask & (1 << id)) != 0]
            self.motor_combo.blockSignals(True)
            self.motor_combo.clear()
            self.motor_combo.addItems([str(id) for id in found])
            self.motor_combo.blockSignals(False)
            self.last_message = "found motor IDs: " + ", ".join(str(id) for id in found)

        if self.motor_combo.currentText() != str(sample.motor_id):
            self.motor_combo.blockSignals(True)
            if self.motor_combo.findText(str(sample.motor_id)) < 0:
                self.motor_combo.addItem(str(sample.motor_id))
            self.motor_combo.setCurrentText(str(sample.motor_id))
            self.motor_combo.blockSignals(False)

        if self.pending_clip is not None:
            if sample.penny_clip == self.pending_clip or time.monotonic() > self.pending_clip_until:
                self.pending_clip = None

        if self.pending_clip is None and not self.clip.hasFocus() and self.clip.value() != sample.penny_clip:
            self.clip.blockSignals(True)
            self.clip.setValue(sample.penny_clip)
            self.clip.blockSignals(False)

        if self.pending_gain is not None:
            if sample.gain_code == self.pending_gain or time.monotonic() > self.pending_gain_until:
                self.pending_gain = None

        if self.pending_gain is None and not self.gain.hasFocus() and self.gain.value() != sample.gain_code:
            self.gain.blockSignals(True)
            self.gain.setValue(sample.gain_code)
            self.gain.blockSignals(False)

    def adc_clip_text(self, flags: int) -> str:
        if flags == 0:
            return "none"
        names = [name for bit, name in enumerate(ADC_CHANNEL_NAMES) if (flags & (1 << bit)) != 0]
        extra = flags & ~((1 << len(ADC_CHANNEL_NAMES)) - 1)
        if extra:
            names.append(f"0x{extra:02x}")
        return ",".join(names)

    def status_text(self, latest: Sample) -> str:
        return (
            f"port={self.port} | PennyESC id={latest.motor_id} | "
            f"speed={latest.motor_speed_rad_s:.3f} rad/s | "
            f"torque_z={self.zeroed_torque(latest):.5f} Nm | "
            f"gain={latest.gain_code} | adc_clip={self.adc_clip_text(latest.clip_flags)} | "
            f"warn=0x{latest.warn_flags:02x} | "
            f"isr={latest.penny_isr_us}/{latest.penny_isr_max_us} us "
            f"i2c={latest.penny_i2c_us} us mct={latest.penny_mct_fault_count} | {self.last_message}"
        )

    def close(self) -> None:
        self.timer.stop()
        self.win.close()
        self.app.quit()

    def exec(self) -> int:
        return self.app.exec()


def find_serial_port(pattern: str | None) -> str:
    if pattern:
        matches = sorted(glob(pattern))
        return matches[0] if matches else pattern

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

        ranked = [item for item in ports if score(item)[0] >= 0]
        if ranked:
            return sorted(ranked, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for candidate in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(candidate))
        if matches:
            return matches[0]

    raise SystemExit("No serial port found")


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def make_test_frame() -> bytes:
    values = [
        SYNC,
        7,
        123456,
        0.125,
        42.0,
        12,
        3,
        0x0008,
        150,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        3,
        0x12,
        0x34,
        0,
    ]
    frame = bytearray(PACKET.pack(*values))
    checksum = 0
    for value in frame[:-1]:
        checksum ^= value
    frame[-1] = checksum
    return bytes(frame)


def self_test() -> int:
    parser = FrameParser()
    samples = parser.append(b"junk" + make_test_frame())
    if len(samples) != 1:
        raise SystemExit("parser self-test failed: no sample")
    sample = samples[0]
    if sample.seq != 7 or sample.motor_id != 3 or sample.set_duty != 12:
        raise SystemExit("parser self-test failed: wrong decoded fields")
    print(f"parser self-test ok: packet_size={PACKET_SIZE}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    port = find_serial_port(args.port)
    ser = open_serial(port)
    samples = SampleBuffer()
    messages: queue.Queue[str] = queue.Queue()
    reader = SerialReader(ser, samples, messages)
    reader.start()

    window = PlotWindow(ser, port, samples, messages)
    try:
        return window.exec()
    finally:
        reader.stop()
        reader.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
