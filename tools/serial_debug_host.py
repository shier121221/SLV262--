#!/usr/bin/env python3
"""
Simple serial debug host for the patrol car protocol.

Frame:
    0x55 + msg_id + len + payload + checksum

Checksum:
    low8(msg_id + len + sum(payload))
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional


HEAD = 0x55

MSG_CAR_STATUS = 0x01
MSG_ENV_BATTERY = 0x02
MSG_TRACK_STATUS = 0x03
MSG_THERMAL_SUMMARY = 0x04
MSG_THERMAL_ROW = 0x05
MSG_COMMAND = 0x10
MSG_PARAM_SET = 0x11
MSG_FAULT_EVENT = 0x12


MSG_NAMES = {
    MSG_CAR_STATUS: "CAR",
    MSG_ENV_BATTERY: "ENV",
    MSG_TRACK_STATUS: "TRACK",
    MSG_THERMAL_SUMMARY: "TH_SUM",
    MSG_THERMAL_ROW: "TH_ROW",
    MSG_COMMAND: "CMD",
    MSG_PARAM_SET: "PARAM",
    MSG_FAULT_EVENT: "FAULT",
}


@dataclass
class ThermalFrame:
    frame_id: int
    tick_ms: int
    rows: Dict[int, list[float]] = field(default_factory=dict)
    first_time: float = field(default_factory=time.time)

    def add_row(self, row: int, temps: list[float]) -> bool:
        self.rows[row] = temps
        return len(self.rows) >= 24


class Parser:
    WAIT_HEAD = 0
    READ_MSG = 1
    READ_LEN = 2
    READ_PAYLOAD = 3
    READ_CHECKSUM = 4

    def __init__(self, max_payload: int = 80):
        self.max_payload = max_payload
        self.state = self.WAIT_HEAD
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0
        self.bad_checksum = 0
        self.bad_length = 0
        self.frames = 0

    def feed(self, data: bytes) -> Iterable[tuple[int, bytes]]:
        for b in data:
            frame = self._feed_byte(b)
            if frame is not None:
                yield frame

    def _feed_byte(self, b: int) -> Optional[tuple[int, bytes]]:
        if self.state == self.WAIT_HEAD:
            if b == HEAD:
                self.state = self.READ_MSG
            return None

        if self.state == self.READ_MSG:
            self.msg_id = b
            self.checksum = b
            self.state = self.READ_LEN
            return None

        if self.state == self.READ_LEN:
            self.length = b
            self.checksum = (self.checksum + b) & 0xFF
            self.payload.clear()
            if self.length > self.max_payload:
                self.bad_length += 1
                self.state = self.WAIT_HEAD
            elif self.length == 0:
                self.state = self.READ_CHECKSUM
            else:
                self.state = self.READ_PAYLOAD
            return None

        if self.state == self.READ_PAYLOAD:
            self.payload.append(b)
            self.checksum = (self.checksum + b) & 0xFF
            if len(self.payload) >= self.length:
                self.state = self.READ_CHECKSUM
            return None

        if self.state == self.READ_CHECKSUM:
            self.state = self.WAIT_HEAD
            if b != self.checksum:
                self.bad_checksum += 1
                return None
            self.frames += 1
            return self.msg_id, bytes(self.payload)

        self.state = self.WAIT_HEAD
        return None


def u16(x: int) -> int:
    return x & 0xFFFF


def i16(x: int) -> int:
    x &= 0xFFFF
    return x - 0x10000 if x & 0x8000 else x


def decode_car(payload: bytes) -> str:
    if len(payload) != 16:
        return f"CAR bad_len={len(payload)} raw={payload.hex(' ')}"
    tick, state, fault, lap, batt, spd_l, spd_r, yaw = struct.unpack("<IBBBBhhh", payload)
    return (
        f"CAR t={tick}ms state={state} fault={fault} lap={lap} batt={batt}% "
        f"spd=({spd_l},{spd_r})cm/s yaw={yaw / 100:.2f}deg"
    )


def decode_env(payload: bytes) -> str:
    if len(payload) != 12:
        return f"ENV bad_len={len(payload)} raw={payload.hex(' ')}"
    tick, temp, humi, batt_mv, batt_pct, pwr = struct.unpack("<IhhHBB", payload)
    return (
        f"ENV t={tick}ms temp={temp / 100:.2f}C humi={humi / 100:.2f}% "
        f"bat={batt_mv}mV pct={batt_pct}% pwr={pwr}"
    )


def decode_track(payload: bytes) -> str:
    if len(payload) != 24:
        return f"TRACK bad_len={len(payload)} raw={payload.hex(' ')}"
    tick, err, ready, use_analog = struct.unpack_from("<IhBB", payload, 0)
    analog = struct.unpack_from("<8H", payload, 8)
    return (
        f"TRACK t={tick}ms err={err} ready={ready} analog_mode={use_analog} "
        f"a={list(analog)}"
    )


def decode_thermal_summary(payload: bytes) -> str:
    if len(payload) != 14:
        return f"TH_SUM bad_len={len(payload)} raw={payload.hex(' ')}"
    tick, frame_id, max_t, min_t, avg_t, max_x, max_y = struct.unpack("<IHhhhBB", payload)
    return (
        f"TH_SUM t={tick}ms frame={frame_id} max={max_t / 100:.2f}C@({max_x},{max_y}) "
        f"min={min_t / 100:.2f}C avg={avg_t / 100:.2f}C"
    )


def decode_thermal_row(payload: bytes, frames: dict[int, ThermalFrame], print_rows: bool) -> str:
    if len(payload) != 72:
        return f"TH_ROW bad_len={len(payload)} raw={payload.hex(' ')}"

    tick, frame_id, row, width = struct.unpack_from("<IHBB", payload, 0)
    raw = struct.unpack_from("<32H", payload, 8)
    temps = [v * 0.02 for v in raw]

    frame = frames.get(frame_id)
    if frame is None:
        frame = ThermalFrame(frame_id=frame_id, tick_ms=tick)
        frames[frame_id] = frame

    complete = frame.add_row(row, temps)
    if len(frames) > 4:
        oldest = sorted(frames.keys())[0]
        frames.pop(oldest, None)

    if complete:
        max_temp = max(max(r) for r in frame.rows.values())
        min_temp = min(min(r) for r in frame.rows.values())
        frames.pop(frame_id, None)
        return (
            f"TH_FRAME frame={frame_id} complete rows=24 "
            f"min={min_temp:.2f}C max={max_temp:.2f}C"
        )

    if print_rows:
        return f"TH_ROW t={tick}ms frame={frame_id} row={row}/{width} first={temps[0]:.2f}C"
    return ""


def decode_fault(payload: bytes) -> str:
    if len(payload) != 7:
        return f"FAULT bad_len={len(payload)} raw={payload.hex(' ')}"
    tick, code, detail = struct.unpack("<IBH", payload)
    return f"FAULT t={tick}ms code={code} detail={detail}"


def decode_frame(msg_id: int, payload: bytes, thermal_frames: dict[int, ThermalFrame], print_rows: bool) -> str:
    if msg_id == MSG_CAR_STATUS:
        return decode_car(payload)
    if msg_id == MSG_ENV_BATTERY:
        return decode_env(payload)
    if msg_id == MSG_TRACK_STATUS:
        return decode_track(payload)
    if msg_id == MSG_THERMAL_SUMMARY:
        return decode_thermal_summary(payload)
    if msg_id == MSG_THERMAL_ROW:
        return decode_thermal_row(payload, thermal_frames, print_rows)
    if msg_id == MSG_FAULT_EVENT:
        return decode_fault(payload)
    name = MSG_NAMES.get(msg_id, f"0x{msg_id:02X}")
    return f"{name} len={len(payload)} raw={payload.hex(' ')}"


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError:
        print("Missing pyserial. Install: pip install pyserial", file=sys.stderr)
        raise SystemExit(2)

    return serial.Serial(port=port, baudrate=baud, timeout=0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="Patrol car serial debug host")
    ap.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("--print-rows", action="store_true", help="Print every thermal row packet")
    ap.add_argument("--raw", action="store_true", help="Print raw frame hex too")
    args = ap.parse_args()

    ser = open_serial(args.port, args.baud)
    parser = Parser()
    thermal_frames: dict[int, ThermalFrame] = {}

    print(f"open {args.port} @ {args.baud}")
    print("waiting frames...")

    last_stat = time.time()
    try:
        while True:
            data = ser.read(512)
            for msg_id, payload in parser.feed(data):
                if args.raw:
                    print(f"RAW msg=0x{msg_id:02X} len={len(payload)} payload={payload.hex(' ')}")
                text = decode_frame(msg_id, payload, thermal_frames, args.print_rows)
                if text:
                    print(text)

            now = time.time()
            if now - last_stat >= 5.0:
                last_stat = now
                print(
                    f"STAT frames={parser.frames} bad_sum={parser.bad_checksum} "
                    f"bad_len={parser.bad_length}"
                )
    except KeyboardInterrupt:
        print("\nstop")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
