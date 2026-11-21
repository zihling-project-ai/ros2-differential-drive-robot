#!/usr/bin/env python3
from dynamixel_sdk import COMM_SUCCESS, PacketHandler

XM_OPERATING_MODE_ADDRESS = 11
XM_TORQUE_ADDRESS = 64
XM_GOAL_VELOCITY_ADDRESS = 104
XM_PRESENT_VELOCITY_ADDRESS = 128
XM_PRESENT_POSITION_ADDRESS = 132

LEFT_ID = 1
RIGHT_ID = 2
REAR_LEFT_ID = 3
REAR_RIGHT_ID = 4
WHEEL_IDS = (LEFT_ID, RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID)
VELOCITY_CONTROL_MODE = 1

XM_TICK2RAD = 0.001533981

# Existing AICAR Base command limit, not a measured actuator limit.
VELOCITY_COMMAND_LIMIT = 265


def to_int32(value):
    """Interpret a raw unsigned 32-bit value as a signed integer."""
    if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError('Expected an unsigned 32-bit integer')
    if value > 0x7FFFFFFF:
        return value - 0x100000000
    return value


def to_uint32(value):
    """Encode a signed 32-bit integer for the SDK."""
    if type(value) is not int or not -0x80000000 <= value <= 0x7FFFFFFF:
        raise ValueError('Expected a signed 32-bit integer')
    return value & 0xFFFFFFFF


class Wheel_Controller:
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)

        self._verify_velocity_mode()
        # Request a stationary target, but never enable torque during construction.
        self.reset()

    def _check_result(self, motor_id, operation, result, error):
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f'Motor {motor_id}: {operation} failed '
                f'(communication={result}, device_error={error})'
            )

    def _verify_velocity_mode(self):
        # The calibrated hardware configuration is not changed by this driver.
        for motor_id in WHEEL_IDS:
            mode, result, error = self.packet.read1ByteTxRx(
                self.port, motor_id, XM_OPERATING_MODE_ADDRESS
            )
            self._check_result(motor_id, 'read operating mode', result, error)
            if mode != VELOCITY_CONTROL_MODE:
                raise RuntimeError(f'Motor {motor_id}: expected velocity control mode, got {mode}')

    def reset(self):
        """Request a stop; encoder baselines are managed by AICAR Base."""
        errors = []
        for motor_id in WHEEL_IDS:
            try:
                result, error = self.packet.write4ByteTxRx(
                    self.port, motor_id, XM_GOAL_VELOCITY_ADDRESS, 0
                )
                self._check_result(motor_id, 'request stop', result, error)
            except Exception as exc:
                # Attempt to stop every wheel, even if one command fails.
                errors.append(f'Motor {motor_id}: {exc}')
        if errors:
            raise RuntimeError('; '.join(errors))

    def set_torque(self, set_data):
        if type(set_data) is not int or set_data not in (0, 1):
            raise ValueError('Torque must be 0 or 1')
        errors = []
        for motor_id in WHEEL_IDS:
            try:
                result, error = self.packet.write1ByteTxRx(
                    self.port, motor_id, XM_TORQUE_ADDRESS, set_data
                )
                self._check_result(motor_id, 'set torque', result, error)
            except Exception as exc:
                if set_data == 1:
                    raise
                # Attempt each wheel on a torque-off request.
                errors.append(f'Motor {motor_id}: {exc}')
        if errors:
            raise RuntimeError('; '.join(errors))

    def set_wheel(self, set_data):
        try:
            left, right = set_data
        except (TypeError, ValueError) as exc:
            raise ValueError('Expected exactly two wheel velocities') from exc

        for velocity in (left, right):
            if type(velocity) is not int or abs(velocity) > VELOCITY_COMMAND_LIMIT:
                raise ValueError('Wheel velocity must be an integer within the command limit')

        if left == right == 0:
            self.reset()
            return

        # Keep the original left/right mapping and signed direction convention.
        left_raw, right_raw = to_uint32(left), to_uint32(right)
        commands = zip(WHEEL_IDS, (left_raw, right_raw, left_raw, right_raw))
        try:
            for motor_id, raw in commands:
                result, error = self.packet.write4ByteTxRx(
                    self.port, motor_id, XM_GOAL_VELOCITY_ADDRESS, raw
                )
                self._check_result(motor_id, 'set wheel velocity', result, error)
        except Exception as exc:
            # A partial motion command is not success. Request a stop on all wheels.
            try:
                self.reset()
            except Exception as stop_exc:
                raise RuntimeError(f'{exc}; stop also failed: {stop_exc}') from exc
            raise

    def get_wheel(self):
        """Return signed encoder ticks in left-front, right-front, left-rear, right-rear order."""
        ticks = []
        for motor_id in WHEEL_IDS:
            raw, result, error = self.packet.read4ByteTxRx(
                self.port, motor_id, XM_PRESENT_POSITION_ADDRESS
            )
            self._check_result(motor_id, 'read position', result, error)
            ticks.append(to_int32(raw))
        return ticks
