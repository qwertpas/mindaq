#!/usr/bin/env python3.11
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import serial

from ft_raw_stream_check import (
    BLOCK,
    CHANNELS,
    DEFAULT_GAIN_CODE,
    RAW_BYTES,
    SAMPLES_PER_BLOCK,
    SYNC,
    raw24,
    resolve_ft,
    valid,
)


SIGNS = {"-", "0", "+"}


def parse_frame(frame: bytes) -> tuple[int, bytes, int, int, int] | None:
    if not valid(frame):
        return None
    sync, seq, raw_block, gain_code, warn_flags, clip_flags, _checksum = BLOCK.unpack(frame)
    if sync != SYNC:
        return None
    return seq, raw_block, gain_code, warn_flags, clip_flags


def consume_frames(scan: bytearray) -> tuple[list[tuple[int, bytes, int, int, int]], int, int]:
    frames = []
    bad = 0
    skipped = 0
    sync = b"\x5A\xA5\x5A\xA5"
    while True:
        pos = scan.find(sync)
        if pos < 0:
            keep = min(len(scan), len(sync) - 1)
            skipped += len(scan) - keep
            del scan[:len(scan) - keep]
            break
        if pos > 0:
            skipped += pos
            del scan[:pos]
        if len(scan) < BLOCK.size:
            break

        frame = bytes(scan[:BLOCK.size])
        parsed = parse_frame(frame)
        if parsed is None:
            bad += 1
            del scan[0]
            continue
        frames.append(parsed)
        del scan[:BLOCK.size]
    return frames, bad, skipped


def drain_stream(ser: serial.Serial, seconds: float) -> None:
    scan = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        scan.extend(ser.read(8192))
        consume_frames(scan)
    ser.reset_input_buffer()


def capture_average(
    ser: serial.Serial, seconds: float, settle: float, flush: float
) -> dict[str, float | int | None]:
    ser.reset_input_buffer()
    if settle > 0.0:
        time.sleep(settle)
        ser.reset_input_buffer()
    if flush > 0.0:
        drain_stream(ser, flush)

    sums = [0.0] * CHANNELS
    sums_sq = [0.0] * CHANNELS
    samples = 0
    missing = 0
    bad = 0
    skipped = 0
    first_seq = None
    last_seq = None
    gain_code = DEFAULT_GAIN_CODE
    warn_flags = 0
    clip_flags = 0
    scan = bytearray()
    start = time.monotonic()
    end = start + seconds

    while time.monotonic() < end:
        scan.extend(ser.read(8192))
        frames, frame_bad, frame_skipped = consume_frames(scan)
        bad += frame_bad
        skipped += frame_skipped
        if not frames:
            continue

        for seq, raw_block, block_gain_code, block_warn_flags, block_clip_flags in frames:
            gain_code = block_gain_code
            warn_flags |= block_warn_flags
            clip_flags |= block_clip_flags
            if first_seq is None:
                first_seq = seq
            elif last_seq is not None:
                expected = last_seq + SAMPLES_PER_BLOCK
                if seq < expected:
                    first_seq = seq
                    missing = 0
                elif seq > expected:
                    missing += seq - expected
            last_seq = seq

            for offset in range(SAMPLES_PER_BLOCK):
                base = offset * RAW_BYTES
                raw = tuple(
                    raw24(raw_block[base:base + RAW_BYTES], index) for index in range(CHANNELS)
                )
                for index, value in enumerate(raw):
                    sums[index] += value
                    sums_sq[index] += value * value
                samples += 1

    frames, frame_bad, frame_skipped = consume_frames(scan)
    bad += frame_bad
    skipped += frame_skipped
    for seq, raw_block, block_gain_code, block_warn_flags, block_clip_flags in frames:
        gain_code = block_gain_code
        warn_flags |= block_warn_flags
        clip_flags |= block_clip_flags
        if first_seq is None:
            first_seq = seq
        elif last_seq is not None:
            expected = last_seq + SAMPLES_PER_BLOCK
            if seq < expected:
                first_seq = seq
                missing = 0
            elif seq > expected:
                missing += seq - expected
        last_seq = seq
        for offset in range(SAMPLES_PER_BLOCK):
            base = offset * RAW_BYTES
            raw = tuple(raw24(raw_block[base:base + RAW_BYTES], index) for index in range(CHANNELS))
            for index, value in enumerate(raw):
                sums[index] += value
                sums_sq[index] += value * value
            samples += 1

    elapsed = time.monotonic() - start
    row: dict[str, float | int | None] = {
        "elapsed_s": elapsed,
        "samples": samples,
        "sample_rate_hz": samples / elapsed if elapsed > 0.0 else 0.0,
        "first_seq": first_seq,
        "last_seq": last_seq,
        "seq_rate_hz": (
            (last_seq - first_seq + SAMPLES_PER_BLOCK) / elapsed
            if first_seq is not None and last_seq is not None and elapsed > 0.0
            else 0.0
        ),
        "missing_seq": missing,
        "bad_checksum": bad,
        "skipped_bytes": skipped,
        "gain_code": gain_code,
        "warn_flags": warn_flags,
        "clip_flags": clip_flags,
    }
    means = []
    for index in range(CHANNELS):
        mean = sums[index] / samples if samples else 0.0
        var = max(0.0, (sums_sq[index] / samples) - mean * mean) if samples else 0.0
        row[f"raw{index}_mean"] = mean
        row[f"raw{index}_sd"] = math.sqrt(var)
        means.append(round(mean))

    fx, fy, fz, tx, ty, tz = resolve_ft(tuple(means), gain_code)
    row.update({"Fx": fx, "Fy": fy, "Fz": fz, "Tx": tx, "Ty": ty, "Tz": tz})
    return row


def output_path(value: str | None) -> Path:
    if value is not None:
        return Path(value)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("data") / f"plate_weight_grid_{stamp}.csv"


def parse_label(answer: str, index: int) -> tuple[str, str, str] | None:
    text = answer.strip().lower()
    if text in {"q", "quit"}:
        return None
    if text in {"", "u", "unlabeled"}:
        return f"point_{index:03d}", "", ""
    if text in {"c", "center"}:
        return "center", "0", "0"

    parts = text.replace(",", " ").split()
    if len(parts) == 2 and parts[0] in SIGNS and parts[1] in SIGNS:
        x_sign, y_sign = parts
        return f"x{x_sign}_y{y_sign}", x_sign, y_sign
    raise ValueError("enter blank, q, center, or two signs like '+ -'")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--flush", type=float, default=0.25)
    parser.add_argument("--output")
    args = parser.parse_args()

    path = output_path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "capture_index", "position", "x_sign", "y_sign", "timestamp", "elapsed_s", "samples",
        "sample_rate_hz", "first_seq", "last_seq", "seq_rate_hz", "missing_seq", "bad_checksum",
        "skipped_bytes", "gain_code", "warn_flags", "clip_flags",
        "raw0_mean", "raw1_mean", "raw2_mean", "raw3_mean", "raw4_mean", "raw5_mean",
        "raw0_sd", "raw1_sd", "raw2_sd", "raw3_sd", "raw4_sd", "raw5_sd",
        "Fx", "Fy", "Fz", "Tx", "Ty", "Tz",
    )

    print(f"opening {args.port}")
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.05) as ser, path.open(
        "w", newline=""
    ) as file:
        ser.dtr = True
        ser.rts = False
        time.sleep(0.3)
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        file.flush()

        index = 1
        while True:
            answer = input(
                f"\n{index} place 500g, then press Enter. "
                "Optional label: '+ -', '0 +', 'center'. Type q to finish: "
            )
            try:
                parsed = parse_label(answer, index)
            except ValueError as exc:
                print(exc)
                continue
            if parsed is None:
                break
            name, x_sign, y_sign = parsed

            print(
                f"capturing {args.seconds:.2f}s after {args.settle:.2f}s settle "
                f"and {args.flush:.2f}s stream flush..."
            )
            row = capture_average(ser, args.seconds, args.settle, args.flush)
            row.update(
                {
                    "capture_index": index,
                    "position": name,
                    "x_sign": x_sign,
                    "y_sign": y_sign,
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                }
            )
            writer.writerow(row)
            file.flush()
            print(
                f"saved {name}: samples={row['samples']} "
                f"rate={row['sample_rate_hz']:.0f}Hz seq_rate={row['seq_rate_hz']:.0f}Hz "
                f"missing={row['missing_seq']} "
                f"gain=x{1 << int(row['gain_code'])} sat=0x{int(row['warn_flags']):02X}/0x{int(row['clip_flags']):02X} "
                f"raw={[round(row[f'raw{i}_mean']) for i in range(CHANNELS)]} "
                f"ft=({row['Fx']:+.3f}, {row['Fy']:+.3f}, {row['Fz']:+.3f}, "
                f"{row['Tx']:+.4f}, {row['Ty']:+.4f}, {row['Tz']:+.4f})"
            )
            index += 1

    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
