#!/usr/bin/env python3
"""
Receive raw IMU samples streamed over BLE NUS from OpenStride.

Usage:
    uv run scripts/imu_recv.py                     # scan + connect, print to stdout
    uv run scripts/imu_recv.py --out data.csv      # also write CSV
    uv run scripts/imu_recv.py --addr AA:BB:CC:... # connect to specific device

Packet format (little-endian, 28 bytes/sample, 8 samples/notify):
    uint32  ts_ms       milliseconds since boot
    float   accel_x/y/z  m/s²
    float   gyro_x/y/z   rad/s

Install deps: pip install bleak
"""

import argparse
import asyncio
import csv
import struct
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakCharacteristicNotFoundError

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device TX → we subscribe

SAMPLE_FMT  = "<Iffffff"   # uint32 + 6 floats
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 28 bytes

assert SAMPLE_SIZE == 28


def parse_args():
    p = argparse.ArgumentParser(description="OpenStride IMU receiver")
    p.add_argument("--addr", help="BLE address (skip scan)")
    p.add_argument("--out",  help="CSV output file")
    p.add_argument("--name", default="OpenStride", help="Device name to scan for")
    return p.parse_args()


class Receiver:
    def __init__(self, out_path):
        self._buf     = bytearray()
        self._csv     = None
        self._writer  = None
        self._count   = 0
        self._t0      = None

        if out_path:
            self._csv    = open(out_path, "w", newline="")
            self._writer = csv.writer(self._csv)
            self._writer.writerow(["ts_ms", "ax", "ay", "az", "gx", "gy", "gz"])
            print(f"Writing to {out_path}", file=sys.stderr)

    def handle(self, _sender, data: bytearray):
        self._buf.extend(data)
        while len(self._buf) >= SAMPLE_SIZE:
            chunk = self._buf[:SAMPLE_SIZE]
            del self._buf[:SAMPLE_SIZE]
            ts_ms, ax, ay, az, gx, gy, gz = struct.unpack(SAMPLE_FMT, chunk)

            if self._t0 is None:
                self._t0 = ts_ms

            if self._writer:
                self._writer.writerow([ts_ms, ax, ay, az, gx, gy, gz])

            self._count += 1
            if self._count % 104 == 1:  # print ~4/sec at 416 Hz
                print(
                    f"t={ts_ms/1000:8.3f}s  "
                    f"a=({ax:+6.2f},{ay:+6.2f},{az:+6.2f}) m/s²  "
                    f"g=({gx:+6.2f},{gy:+6.2f},{gz:+6.2f}) rad/s  "
                    f"[{self._count} samples]"
                )

    def close(self):
        if self._csv:
            self._csv.close()


async def find_device(name):
    print(f"Scanning for '{name}'...", file=sys.stderr)
    device = await BleakScanner.find_device_by_name(name, timeout=10.0)
    if device is None:
        print(f"Device '{name}' not found", file=sys.stderr)
        sys.exit(1)
    print(f"Found {device.name} @ {device.address}", file=sys.stderr)
    return device.address


async def run(args):
    addr = args.addr or await find_device(args.name)
    recv = Receiver(args.out)

    print(f"Connecting to {addr}...", file=sys.stderr)
    async with BleakClient(addr) as client:
        print(f"Connected (MTU={client.mtu_size})", file=sys.stderr)

        services = [str(s.uuid) for s in client.services]
        print(f"Services: {', '.join(services)}", file=sys.stderr)

        try:
            await client.start_notify(NUS_TX_CHAR, recv.handle)
        except BleakCharacteristicNotFoundError:
            print(
                "NUS TX characteristic not found — firmware without NUS flashed?\n"
                f"  Expected: {NUS_TX_CHAR}\n"
                f"  Present services: {', '.join(services)}",
                file=sys.stderr,
            )
            return

        print("Streaming — Ctrl-C to stop", file=sys.stderr)
        try:
            await asyncio.get_event_loop().create_future()  # run until cancelled
        except asyncio.CancelledError:
            pass
        finally:
            await client.stop_notify(NUS_TX_CHAR)
            recv.close()


if __name__ == "__main__":
    args = parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
