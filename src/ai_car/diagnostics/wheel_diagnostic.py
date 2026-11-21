#!/usr/bin/env python3
"""Manual, read-only wheel feedback diagnostic; never commands motion."""

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from ai_car.dxl_wheel import (
    WHEEL_IDS,
    XM_PRESENT_POSITION_ADDRESS,
    XM_PRESENT_VELOCITY_ADDRESS,
    to_int32,
)

DEVICE = '/dev/U2D2'
BAUD_RATE = 1000000


def main():
    port = PortHandler(DEVICE)
    packet = PacketHandler(1.0)
    if not port.openPort():
        raise RuntimeError(f'Cannot open {DEVICE}')
    try:
        if not port.setBaudRate(BAUD_RATE):
            raise RuntimeError('Cannot configure baud rate')

        # Read feedback directly; constructing a controller would request a stop.
        for motor_id in WHEEL_IDS:
            readings = []
            for address in (XM_PRESENT_POSITION_ADDRESS, XM_PRESENT_VELOCITY_ADDRESS):
                raw, result, error = packet.read4ByteTxRx(port, motor_id, address)
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f'Motor {motor_id}: read failed '
                        f'(communication={result}, device_error={error})'
                    )
                readings.append(to_int32(raw))
            position, velocity = readings
            print(f'Motor {motor_id}: position={position} ticks, velocity={velocity} raw units')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
