#!/usr/bin/env python3.11
import argparse
import struct
import time

import serial


SYNC = 0xA55AA55A
SYNC_BYTES = b"\x5a\xa5\x5a\xa5"
PACKET = struct.Struct("<II18sB")
RAW = bytes(
    byte
    for value in (0x00101010 + i * 0x00010101 for i in range(6))
    for byte in (value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF)
)


def valid(frame: bytes) -> bool:
    check = 0
    for byte in frame[:-1]:
        check ^= byte
    return check == frame[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem1101")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--baud", type=int, default=2_000_000)
    args = parser.parse_args()

    samples = 0
    gaps = 0
    bad = 0
    skipped = 0
    restarts = 0
    bytes_read = 0
    first_seq = None
    last_seq = None
    buffer = bytearray()

    with serial.Serial(args.port, baudrate=args.baud, timeout=0.05) as ser:
      ser.dtr = True
      ser.rts = False
      time.sleep(0.3)
      ser.reset_input_buffer()
      while first_seq is None:
          chunk = ser.read(4096)
          if not chunk:
              continue
          buffer.extend(chunk)
          while len(buffer) >= PACKET.size:
              if buffer[:4] != SYNC_BYTES:
                  index = buffer.find(SYNC_BYTES, 1)
                  if index < 0:
                      skipped += max(0, len(buffer) - 3)
                      del buffer[:-3]
                      break
                  skipped += index
                  del buffer[:index]
                  continue
              frame = bytes(buffer[:PACKET.size])
              if not valid(frame):
                  del buffer[0]
                  continue
              sync, seq, raw, _checksum = PACKET.unpack(frame)
              if sync != SYNC or raw != RAW:
                  del buffer[0]
                  continue
              first_seq = seq
              last_seq = seq
              samples = 1
              del buffer[:PACKET.size]
              break

      start = time.monotonic()
      end = start + args.seconds
      while time.monotonic() < end:
          chunk = ser.read(65536)
          if not chunk:
              continue
          bytes_read += len(chunk)
          buffer.extend(chunk)

          while len(buffer) >= PACKET.size:
              if buffer[:4] != SYNC_BYTES:
                  index = buffer.find(SYNC_BYTES, 1)
                  if index < 0:
                      skipped += max(0, len(buffer) - 3)
                      del buffer[:-3]
                      break
                  skipped += index
                  del buffer[:index]
                  continue

              frame = bytes(buffer[:PACKET.size])
              if not valid(frame):
                  del buffer[0]
                  bad += 1
                  continue

              sync, seq, raw, _checksum = PACKET.unpack(frame)
              if sync != SYNC or raw != RAW:
                  del buffer[0]
                  bad += 1
                  continue

              if first_seq is None:
                  first_seq = seq
              if last_seq is not None and seq != last_seq + 1:
                  if seq > last_seq:
                      gaps += seq - last_seq - 1
                  else:
                      restarts += 1
              last_seq = seq
              samples += 1
              del buffer[:PACKET.size]

    elapsed = time.monotonic() - start
    print(f"port={args.port}")
    print(f"packet_bytes={PACKET.size}")
    print(f"samples={samples}")
    print(f"sample_rate_hz={samples / elapsed:.1f}")
    print(f"bytes_per_s={bytes_read / elapsed:.0f}")
    print(f"mbit_per_s={(bytes_read * 8.0) / elapsed / 1_000_000.0:.2f}")
    print(f"first_seq={first_seq}")
    print(f"last_seq={last_seq}")
    print(f"missing_seq={gaps}")
    print(f"restarts={restarts}")
    print(f"skipped_bytes={skipped}")
    print(f"bad_checksum={bad}")
    return 0 if samples >= 31000 * args.seconds and gaps == 0 and bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
