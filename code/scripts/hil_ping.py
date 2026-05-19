#!/usr/bin/env python3

import argparse
import struct
import time
from dataclasses import dataclass

import serial


SENSOR_MAGIC = 0x5348   # bytes: H S
CONTROL_MAGIC = 0x4348  # bytes: H C
PROTOCOL_VERSION = 1

SENSOR_VALID_ENCODERS = 1 << 0
SENSOR_VALID_IMU = 1 << 1
SENSOR_VALID_COMMAND = 1 << 2

CONTROL_LEFT_MOTOR_MISSING = 1 << 0
CONTROL_RIGHT_MOTOR_MISSING = 1 << 1
CONTROL_INVALID_DT = 1 << 2
CONTROL_CONTROLLER_FAULT = 1 << 3

SENSOR_FMT = "<HBHIIBqqffffH"
CONTROL_FMT = "<HBHffffBH"

SENSOR_SIZE = struct.calcsize(SENSOR_FMT)
CONTROL_SIZE = struct.calcsize(CONTROL_FMT)

assert SENSOR_SIZE == 48, SENSOR_SIZE
assert CONTROL_SIZE == 24, CONTROL_SIZE


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


@dataclass
class SensorFrame:
    seq: int
    time_us: int
    dt_us: int
    valid_mask: int
    left_encoder_ticks: int
    right_encoder_ticks: int
    imu_yaw_rad: float
    imu_omega_z_rad_s: float
    target_v_mps: float
    target_w_rad_s: float

    def pack(self) -> bytes:
        without_crc = struct.pack(
            SENSOR_FMT[:-1],
            SENSOR_MAGIC,
            PROTOCOL_VERSION,
            self.seq & 0xFFFF,
            self.time_us & 0xFFFFFFFF,
            self.dt_us & 0xFFFFFFFF,
            self.valid_mask & 0xFF,
            self.left_encoder_ticks,
            self.right_encoder_ticks,
            self.imu_yaw_rad,
            self.imu_omega_z_rad_s,
            self.target_v_mps,
            self.target_w_rad_s,
        )

        crc = crc16_ccitt_false(without_crc)

        return without_crc + struct.pack("<H", crc)


@dataclass
class ControlFrame:
    seq: int
    left_motor_voltage: float
    right_motor_voltage: float
    debug_v_mps: float
    debug_w_rad_s: float
    status: int


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()

    while len(data) < size:
        chunk = port.read(size - len(data))

        if not chunk:
            raise TimeoutError(f"timeout while reading {size} bytes")

        data.extend(chunk)

    return bytes(data)


def sync_to_magic(port: serial.Serial, magic: int) -> bytes:
    magic_lo = magic & 0xFF
    magic_hi = (magic >> 8) & 0xFF

    matched_low = False

    while True:
        b = read_exact(port, 1)[0]

        if not matched_low:
            matched_low = b == magic_lo
            continue

        if b == magic_hi:
            return bytes([magic_lo, magic_hi])

        matched_low = b == magic_lo


def read_control_frame(port: serial.Serial) -> ControlFrame:
    prefix = sync_to_magic(port, CONTROL_MAGIC)
    rest = read_exact(port, CONTROL_SIZE - len(prefix))

    packet = prefix + rest

    fields = struct.unpack(CONTROL_FMT, packet)

    magic = fields[0]
    version = fields[1]
    seq = fields[2]
    left_motor_voltage = fields[3]
    right_motor_voltage = fields[4]
    debug_v_mps = fields[5]
    debug_w_rad_s = fields[6]
    status = fields[7]
    received_crc = fields[8]

    if magic != CONTROL_MAGIC:
        raise ValueError(f"bad magic: 0x{magic:04x}")

    if version != PROTOCOL_VERSION:
        raise ValueError(f"bad protocol version: {version}")

    calculated_crc = crc16_ccitt_false(packet[:-2])

    if received_crc != calculated_crc:
        raise ValueError(
            f"bad crc: received=0x{received_crc:04x}, "
            f"calculated=0x{calculated_crc:04x}"
        )

    return ControlFrame(
        seq=seq,
        left_motor_voltage=left_motor_voltage,
        right_motor_voltage=right_motor_voltage,
        debug_v_mps=debug_v_mps,
        debug_w_rad_s=debug_w_rad_s,
        status=status,
    )


def describe_status(status: int) -> str:
    if status == 0:
        return "OK"

    flags = []

    if status & CONTROL_LEFT_MOTOR_MISSING:
        flags.append("LEFT_MOTOR_MISSING")

    if status & CONTROL_RIGHT_MOTOR_MISSING:
        flags.append("RIGHT_MOTOR_MISSING")

    if status & CONTROL_INVALID_DT:
        flags.append("INVALID_DT")

    if status & CONTROL_CONTROLLER_FAULT:
        flags.append("CONTROLLER_FAULT")

    unknown = status & ~(
        CONTROL_LEFT_MOTOR_MISSING
        | CONTROL_RIGHT_MOTOR_MISSING
        | CONTROL_INVALID_DT
        | CONTROL_CONTROLLER_FAULT
    )

    if unknown:
        flags.append(f"UNKNOWN=0x{unknown:02x}")

    return "|".join(flags)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send HIL SensorFrame packets to STM32 and read ControlFrame responses."
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial device, for example /dev/ttyACM0 or /dev/ttyUSB0.",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART baudrate.",
    )

    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="Number of frames to send.",
    )

    parser.add_argument(
        "--period-ms",
        type=int,
        default=32,
        help="HIL frame period in milliseconds.",
    )

    parser.add_argument(
        "--v",
        type=float,
        default=0.0,
        help="Target linear velocity in m/s.",
    )

    parser.add_argument(
        "--w",
        type=float,
        default=0.0,
        help="Target angular velocity in rad/s.",
    )

    args = parser.parse_args()

    valid_mask = (
        SENSOR_VALID_ENCODERS
        | SENSOR_VALID_IMU
        | SENSOR_VALID_COMMAND
    )

    dt_us = args.period_ms * 1000

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.5,
        write_timeout=0.5,
    ) as port:
        port.reset_input_buffer()
        port.reset_output_buffer()

        print(
            f"opened {args.port} at {args.baud}, "
            f"SensorFrame={SENSOR_SIZE} bytes, "
            f"ControlFrame={CONTROL_SIZE} bytes"
        )

        start = time.monotonic()

        for i in range(args.count):
            now = time.monotonic()
            time_us = int((now - start) * 1_000_000)

            frame = SensorFrame(
                seq=i,
                time_us=time_us,
                dt_us=dt_us,
                valid_mask=valid_mask,
                left_encoder_ticks=0,
                right_encoder_ticks=0,
                imu_yaw_rad=0.0,
                imu_omega_z_rad_s=0.0,
                target_v_mps=args.v,
                target_w_rad_s=args.w,
            )

            packet = frame.pack()

            port.write(packet)
            port.flush()

            try:
                response = read_control_frame(port)
            except Exception as exc:
                print(f"[{i:04d}] FAIL: {exc}")
                time.sleep(args.period_ms / 1000.0)
                continue

            seq_ok = response.seq == (i & 0xFFFF)
            status_text = describe_status(response.status)

            print(
                f"[{i:04d}] "
                f"seq={response.seq} "
                f"seq_ok={seq_ok} "
                f"status={status_text} "
                f"ul={response.left_motor_voltage:.4f} "
                f"ur={response.right_motor_voltage:.4f} "
                f"v_dbg={response.debug_v_mps:.4f} "
                f"w_dbg={response.debug_w_rad_s:.4f}"
            )

            time.sleep(args.period_ms / 1000.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
