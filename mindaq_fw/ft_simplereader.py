#!/usr/bin/env python3.11
import struct
import serial
import time

SERIAL_PORT = "/dev/cu.usbmodem1101"


''' Copy this to read force/torque from the serial port '''
def read_force_torque(ser: serial.Serial) -> tuple[float, float, float, float, float, float]:
    SYNC = 0xA55A
    PACKET = struct.Struct("<HI6fB")
    PACKET_SIZE = PACKET.size
    ser.reset_input_buffer()
    while True:
        if ser.read(1) != b"\x5a":
            continue
        if ser.read(1) != b"\xa5":
            continue
        frame = b"\x5a\xa5" + ser.read(PACKET_SIZE - 2)
        checksum = 0
        for value in frame[:-1]:
            checksum ^= value
        if checksum != frame[-1]:
            continue
        sync, _timestamp_us, *values, _checksum = PACKET.unpack(frame)
        if sync == SYNC:
            return tuple(values)



if __name__ == "__main__":
    ser = serial.Serial(SERIAL_PORT, baudrate=2_000_000)
    
    while True:
        fx, fy, fz, tx, ty, tz = read_force_torque(ser)

        print(f"Fx {fx:+7.3f} N  Fy {fy:+7.3f} N  Fz {fz:+7.3f} N  Tx {tx:+7.3f} Nm  Ty {ty:+7.3f} Nm  Tz {tz:+7.3f} Nm")
        time.sleep(0.01)
