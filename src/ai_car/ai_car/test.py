#!/usr/bin/env python3
"""Read-only DYNAMIXEL AX-series Joint 2 diagnostic."""

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICE = '/dev/U2D2'
BAUD_RATE = 1000000
PROTOCOL_VERSION = 1.0
MOTOR_ID = 12
EXPECTED_MODEL = 12

REGISTERS = [
    ('Torque Enable', 24, 1),
    ('Goal Position', 30, 2),
    ('Moving Speed', 32, 2),
    ('Torque Limit', 34, 2),
    ('Present Position', 36, 2),
    ('Present Load', 40, 2),
    ('Present Voltage', 42, 1),
    ('Present Temperature', 43, 1),
    ('Moving', 46, 1),
]


def check_result(packet, result, error, operation):
    if result != COMM_SUCCESS:
        raise RuntimeError(f'{operation}: {packet.getTxRxResult(result)}')
    if error != 0:
        raise RuntimeError(f'{operation}: {packet.getRxPacketError(error)}')


def read_register(packet, port, motor_id, address, size):
    if size == 1:
        value, result, error = packet.read1ByteTxRx(port, motor_id, address)
    elif size == 2:
        value, result, error = packet.read2ByteTxRx(port, motor_id, address)
    else:
        raise ValueError(f'Unsupported register width: {size}')
    check_result(packet, result, error, f'ID {motor_id} read address {address}')
    return value


def main():
    port = PortHandler(DEVICE)
    packet = PacketHandler(PROTOCOL_VERSION)
    opened = False
    try:
        if not port.openPort():
            raise RuntimeError(f'Cannot open {DEVICE}')
        opened = True
        if not port.setBaudRate(BAUD_RATE):
            raise RuntimeError(f'Cannot set baud rate {BAUD_RATE}')
        print('[PASS] U2D2 connected')

        model, result, error = packet.ping(port, MOTOR_ID)
        check_result(packet, result, error, f'ID {MOTOR_ID} ping')
        if model != EXPECTED_MODEL:
            raise RuntimeError(f'Unexpected model: {model} (expected {EXPECTED_MODEL})')
        print(f'[PASS] Motor ID {MOTOR_ID}, Model {model}')
        print('\n--- Arm Diagnostic (READ ONLY) ---')

        values = {}
        for name, address, size in REGISTERS:
            value = read_register(packet, port, MOTOR_ID, address, size)
            values[name] = value
            if name == 'Present Voltage':
                print(f'{name}: {value / 10:.1f} V (raw {value})')
            elif name == 'Present Temperature':
                print(f'{name}: {value} °C')
            else:
                print(f'{name}: {value}')

        goal = values['Goal Position']
        present = values['Present Position']
        print('\n--- Position Comparison ---')
        print(f'Goal - Present: {goal - present} Tick')
        print(f'Absolute Error: {abs(goal - present)} Tick')
        print(f'Approx. Angle Error: {abs(goal - present) * 0.29296875:.2f} deg')
        if values['Torque Limit'] == 0:
            print('[WARN] Torque Limit is zero; do not issue motion commands.')
        if values['Moving'] != 0:
            print('[WARN] Motor reports moving; check physical state.')
        print('[INFO] No write commands were sent')
    except Exception as exc:
        print(f'[ERROR] {exc}')
    finally:
        if opened:
            port.closePort()
            print('[INFO] Port closed')


if __name__ == '__main__':
    main()



'''
import time
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# ============================================================
# P4-5 Four-Wheel Control Test
# ============================================================

DEVICE = '/dev/U2D2'
BAUD_RATE = 1000000
PROTOCOL_VERSION = 1.0

WHEEL_IDS = [1, 2, 3, 4]

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

TEST_VELOCITY = 100
TEST_DURATION = 0.5

STOP_THRESHOLD = 2

port = PortHandler(DEVICE)
packet = PacketHandler(PROTOCOL_VERSION)

port_opened = False
motion_attempted = False


# ============================================================
# Helper Functions
# ============================================================

def to_int32(value):
    if value > 0x7FFFFFFF:
        return value - 0x100000000
    return value


def to_uint32(value):
    return int(value) & 0xFFFFFFFF


def check_result(result, error, operation):
    if result != COMM_SUCCESS:
        raise RuntimeError(
            f'{operation}: {packet.getTxRxResult(result)}'
        )

    if error != 0:
        raise RuntimeError(
            f'{operation}: {packet.getRxPacketError(error)}'
        )


def read1(motor_id, address):
    value, result, error = packet.read1ByteTxRx(
        port, motor_id, address
    )
    check_result(result, error, f'ID {motor_id} read1')
    return value


def read4(motor_id, address):
    value, result, error = packet.read4ByteTxRx(
        port, motor_id, address
    )
    check_result(result, error, f'ID {motor_id} read4')
    return to_int32(value)


def write_velocity(motor_id, velocity):
    result, error = packet.write4ByteTxRx(
        port,
        motor_id,
        ADDR_GOAL_VELOCITY,
        to_uint32(velocity)
    )

    check_result(
        result,
        error,
        f'ID {motor_id} write velocity'
    )


def stop_all_wheels():
    """
    Attempt to stop every wheel even if one write fails.
    """
    all_ok = True

    for motor_id in WHEEL_IDS:
        try:
            write_velocity(motor_id, 0)
            print(f'[STOP] ID {motor_id}: zero command sent')

        except Exception as e:
            all_ok = False
            print(f'[CRITICAL] ID {motor_id} stop failed: {e}')

    return all_ok


def verify_stopped():
    """
    Read back the present velocity of all four wheels.
    """
    all_stopped = True

    for motor_id in WHEEL_IDS:
        try:
            velocity = read4(
                motor_id,
                ADDR_PRESENT_VELOCITY
            )

            print(
                f'[AFTER] ID {motor_id} '
                f'Present Velocity: {velocity}'
            )

            if abs(velocity) > STOP_THRESHOLD:
                all_stopped = False

        except Exception as e:
            all_stopped = False
            print(
                f'[CRITICAL] ID {motor_id} '
                f'velocity read failed: {e}'
            )

    return all_stopped


# ============================================================
# Main Test
# ============================================================

try:
    # P1: Open U2D2
    if not port.openPort():
        raise RuntimeError('Cannot open U2D2')

    port_opened = True

    if not port.setBaudRate(BAUD_RATE):
        raise RuntimeError('Cannot set baud rate')

    print('[PASS] U2D2 connected')

    # P2: Preflight checks for all four motors
    print('\n--- Preflight Checks ---')

    for motor_id in WHEEL_IDS:

        model, result, error = packet.ping(
            port, motor_id
        )
        check_result(result, error, f'ID {motor_id} ping')

        if model != 1060:
            raise RuntimeError(
                f'ID {motor_id}: unexpected model {model}'
            )

        mode = read1(
            motor_id,
            ADDR_OPERATING_MODE
        )

        torque = read1(
            motor_id,
            ADDR_TORQUE_ENABLE
        )

        velocity = read4(
            motor_id,
            ADDR_PRESENT_VELOCITY
        )

        print(
            f'[CHECK] ID {motor_id}: '
            f'Model={model}, '
            f'Mode={mode}, '
            f'Torque={torque}, '
            f'Velocity={velocity}'
        )

        if mode != 1:
            raise RuntimeError(
                f'ID {motor_id}: not in Velocity Mode'
            )

        if torque != 1:
            raise RuntimeError(
                f'ID {motor_id}: torque disabled'
            )

        if abs(velocity) > STOP_THRESHOLD:
            raise RuntimeError(
                f'ID {motor_id}: motor is not stationary'
            )

    print('[PASS] All four motors ready')

    # P3: Record initial encoder positions
    positions_before = {}

    print('\n--- Before Motion ---')

    for motor_id in WHEEL_IDS:
        position = read4(
            motor_id,
            ADDR_PRESENT_POSITION
        )

        positions_before[motor_id] = position

        print(
            f'[BEFORE] ID {motor_id}: '
            f'Position={position}'
        )

    # P4: Four-wheel motion test
    print('\n--- Four-Wheel Motion Test ---')

    print(f'[COMMAND] Velocity={TEST_VELOCITY}')
    print(f'[COMMAND] Duration={TEST_DURATION}s')

    motion_attempted = True
    try:
        for motor_id in WHEEL_IDS:
            write_velocity(
                motor_id,
                TEST_VELOCITY
            )

            print(
                f'[RUN] ID {motor_id}: '
                f'Goal Velocity={TEST_VELOCITY}'
            )

        time.sleep(TEST_DURATION)

    finally:
        stop_all_wheels()

    # P5: Wait for deceleration
    time.sleep(0.5)

    print('\n--- Stop Verification ---')

    if verify_stopped():
        print('[PASS] All wheels report near-zero velocity')
    else:
        print('[FAIL] One or more wheels not verified stopped')

    # P6: Encoder Readback
    print('\n--- Encoder Delta ---')

    for motor_id in WHEEL_IDS:

        position_after = read4(
            motor_id,
            ADDR_PRESENT_POSITION
        )

        delta = (
            position_after -
            positions_before[motor_id]
        )

        print(
            f'[RESULT] ID {motor_id}: '
            f'Before={positions_before[motor_id]}, '
            f'After={position_after}, '
            f'Delta={delta} Tick'
        )

except KeyboardInterrupt:
    print('\n[INTERRUPTED] Test cancelled')

except Exception as e:
    print(f'[ERROR] {e}')

finally:
    # Attempt to stop all motors after any motion attempt
    if port_opened:
        if motion_attempted:
            print('\n--- Final Safety Stop ---')
            stop_all_wheels()

        port.closePort()
        print('[INFO] Port closed')
'''