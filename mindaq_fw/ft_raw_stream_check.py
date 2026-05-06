#!/usr/bin/env python3.11
import argparse
import struct
import time

import serial


SYNC = 0xA55AA55A
SAMPLE_RATE_HZ = 32_000.0
CHANNELS = 6
SAMPLES_PER_BLOCK = 64
RAW_BYTES = CHANNELS * 3
BLOCK = struct.Struct(f"<II{SAMPLES_PER_BLOCK * RAW_BYTES}sB")
ADC_CODE_TO_NET_GAUGE = 0.016
NET_GAUGE_ZERO = 32768.0
ADC_ZERO_CODE = (-521738.0, -153570.0, -334040.0, 26775.0, -317685.0, -7511.0)
GAUGE_OFFSET = (25212.0, 29526.0, 23086.0, 28152.0, 24516.0, 33182.0)
FT_MATRIX = (
    (9.575586391247820e-06, 5.034719323341890e-04, 2.827122413565440e-05,
     -5.179287349565690e-04, -6.054530989079390e-06, -2.075418832354940e-05),
    (-1.038902452683050e-07, -2.775572866673270e-04, 8.796242351000440e-06,
     -3.247148267647300e-04, -2.520127206219610e-05, 6.348497973460170e-04),
    (5.498814948547810e-04, 4.982665178403360e-05, 5.962241921681900e-04,
     1.831416126079740e-05, 4.950476642689660e-04, 4.274870927968580e-05),
    (-3.124957530622520e-06, 1.381712094274260e-06, 3.204207898418680e-06,
     2.084386424212550e-06, 2.577052836577890e-07, -3.830766465429980e-06),
    (1.741821139441800e-06, 3.219636663939340e-06, 2.229410518476670e-06,
     -3.070929538864320e-06, -3.172020201011310e-06, -4.085733997308220e-07),
    (2.397902398001370e-08, 2.162107308175090e-06, -7.705459896268520e-08,
     2.297459226084220e-06, -9.943845273683800e-08, 2.240688262268020e-06),
)


def valid(frame: bytes) -> bool:
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


def resolve_ft(raw: tuple[int, ...]) -> tuple[float, ...]:
    gages = []
    for index, value in enumerate(raw):
        net_gage = (float(value) - ADC_ZERO_CODE[index]) * ADC_CODE_TO_NET_GAUGE + NET_GAUGE_ZERO
        gages.append(net_gage - GAUGE_OFFSET[index])
    return tuple(sum(row[col] * gages[col] for col in range(CHANNELS)) for row in FT_MATRIX)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--baud", type=int, default=2_000_000)
    args = parser.parse_args()

    with serial.Serial(args.port, baudrate=args.baud, timeout=0.05) as ser:
        ser.dtr = False
        ser.rts = False
        time.sleep(5.0)
        ser.reset_input_buffer()

        scan = bytearray()
        start_frame = None
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and start_frame is None:
            scan.extend(ser.read(4096))
            limit = max(0, len(scan) - BLOCK.size + 1)
            for pos in range(limit):
                if scan[pos:pos + 4] != b"\x5A\xA5\x5A\xA5":
                    continue
                frame = bytes(scan[pos:pos + BLOCK.size])
                if valid(frame):
                    start_frame = frame
                    del scan[:pos + BLOCK.size]
                    break
            if len(scan) > BLOCK.size * 4:
                del scan[:-BLOCK.size]

        if start_frame is None:
            capture = bytearray()
            start = time.monotonic()
        else:
            capture = bytearray(start_frame)
            capture.extend(scan)
            start = time.monotonic()

        end = start + args.seconds
        while time.monotonic() < end:
            chunk = ser.read(65536)
            capture.extend(chunk)
        elapsed = time.monotonic() - start

    samples = 0
    missing = 0
    bad = 0
    skipped = 0
    first_seq = None
    last_seq = None
    latest_raw_bytes = None
    resets = 0
    pos = 0
    while pos + BLOCK.size <= len(capture):
        if capture[pos:pos + 4] != b"\x5A\xA5\x5A\xA5":
            pos += 1
            skipped += 1
            continue

        frame = bytes(capture[pos:pos + BLOCK.size])
        if not valid(frame):
            pos += 1
            bad += 1
            continue

        sync, seq, raw_block, _checksum = BLOCK.unpack(frame)
        if sync != SYNC:
            pos += 1
            bad += 1
            continue

        if first_seq is None:
            first_seq = seq
        elif last_seq is not None:
            expected = last_seq + SAMPLES_PER_BLOCK
            if seq < expected:
                resets += 1
                samples = 0
                missing = 0
                bad = 0
                skipped = 0
                first_seq = seq
                last_seq = None
            elif seq > expected:
                missing += seq - expected
        last_seq = seq
        latest_raw_bytes = raw_block[-RAW_BYTES:]
        samples += SAMPLES_PER_BLOCK
        pos += BLOCK.size

    elapsed = max(1e-9, elapsed)
    stream_bytes = (samples // SAMPLES_PER_BLOCK) * BLOCK.size
    if latest_raw_bytes is not None:
        latest_raw = tuple(raw24(latest_raw_bytes, index) for index in range(CHANNELS))
        latest_ft = resolve_ft(latest_raw)
    else:
        latest_raw = None
        latest_ft = None
    print(f"port={args.port}")
    print(f"block_bytes={BLOCK.size}")
    print(f"samples_per_block={SAMPLES_PER_BLOCK}")
    print(f"samples={samples}")
    print(f"sample_rate_hz={samples / elapsed:.1f}")
    print(f"bytes_per_s={stream_bytes / elapsed:.0f}")
    print(f"mbit_per_s={(stream_bytes * 8.0) / elapsed / 1_000_000.0:.2f}")
    print(f"first_seq={first_seq}")
    print(f"last_seq={last_seq}")
    print(f"missing_seq={missing}")
    print(f"skipped_bytes={skipped}")
    print(f"bad_checksum={bad}")
    print(f"sequence_resets={resets}")
    if latest_raw is not None and latest_ft is not None:
        print("raw=" + " ".join(str(value) for value in latest_raw))
        print(
            "ft="
            + " ".join(
                f"{name}={value:+.4f}"
                for name, value in zip(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), latest_ft)
            )
        )
    return 0 if samples >= 31_900 * args.seconds and missing == 0 and bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
