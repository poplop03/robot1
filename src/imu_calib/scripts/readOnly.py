#!/usr/bin/env python3
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
PACKET_SIZE = 28
HEADER = b'\x24\x03'

while True:
    # Sync packet
    while True:
        c = ser.read(1)
        if c == HEADER[0:1] and ser.read(1) == HEADER[1:2]:
            packet = HEADER + ser.read(PACKET_SIZE - 2)
            if len(packet) == PACKET_SIZE:
                break

    print("Raw packet:", list(packet))
    gx = struct.unpack('>h', packet[10:12])[0]
    gy = struct.unpack('>h', packet[12:14])[0]
    gz = struct.unpack('>h', packet[14:16])[0]
    ax = struct.unpack('>h', packet[16:18])[0]
    ay = struct.unpack('>h', packet[18:20])[0]
    az = struct.unpack('>h', packet[20:22])[0]
    print(f"Gyro: {gx}, {gy}, {gz} | Accel: {ax}, {ay}, {az}")