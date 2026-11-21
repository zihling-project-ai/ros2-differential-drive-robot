#!/usr/bin/env python3
"""Manual, read-only arm position diagnostic; not a hardware motion test."""

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from ai_car.dxl_arm import ARM_MOTOR_IDS, AX_PRESENT_POSITION, validate_position_tick

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

        # Do not create an Arm_Controller here: its constructor writes settings.
        for motor_id in ARM_MOTOR_IDS:
            tick, result, error = packet.read2ByteTxRx(
                port, motor_id, AX_PRESENT_POSITION
            )
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(
                    f'Motor {motor_id}: position read failed '
                    f'(communication={result}, device_error={error})'
                )
            validate_position_tick(tick)
            print(f'Motor {motor_id}: present position = {tick} ticks')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
