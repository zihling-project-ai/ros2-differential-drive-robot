#!/usr/bin/env python3

import math
import time
from dynamixel_sdk import COMM_SUCCESS, PacketHandler
from ai_car.constants import (
    HOME_POS, HOME_TOOL, JOINT_MIN_DEG, JOINT_MAX_DEG, TOOL_MIN_DEG, TOOL_MAX_DEG,
)

AX_LED_ADDRESS = 25
AX_TORQUE_ADDRESS = 24
AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36

ARM_JOINT_1 = 11
ARM_JOINT_2 = 12
ARM_TOOL = 13
ARM_MOTOR_IDS = (ARM_JOINT_1, ARM_JOINT_2, ARM_TOOL)

# Preserve the original calibration offsets; physical limits remain unverified.
JOINT_1_ZERO_TICK = 812
JOINT_2_ZERO_TICK = 212
TOOL_ZERO_TICK = 420

AX_MAX_POSITION_TICK = 1023

AX_TICK2DEG = 300.0 / 1024
AX_TICK2RAD = math.radians(AX_TICK2DEG)


def angle_to_ticks(angle_deg, max_angle_deg, min_angle_deg=JOINT_MIN_DEG):
    if isinstance(angle_deg, bool) or not isinstance(angle_deg, (int, float)):
        raise ValueError('Angle must be a number in degrees')
    if not min_angle_deg <= angle_deg <= max_angle_deg:
        raise ValueError(f'Angle must be within {min_angle_deg}–{max_angle_deg} degrees')
    return int(angle_deg / AX_TICK2DEG)


def validate_position_tick(tick):
    if not isinstance(tick, int) or not 0 <= tick <= AX_MAX_POSITION_TICK:
        raise ValueError('Position out of the AX encoder range')

class Arm_Controller:
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)

        for motor_id in ARM_MOTOR_IDS:
            result, error = self.packet.write2ByteTxRx(
                self.port, motor_id, AX_MOVING_SPEED, 100
            )
            self._check_result(motor_id, 'configure speed', result, error)
        time.sleep(1)

    def _check_result(self, motor_id, operation, result, error):
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f'Motor {motor_id}: {operation} failed '
                f'(communication={result}, device_error={error})'
            )

    def reset(self):
        self.set_joint(HOME_POS)
        self.set_tool(HOME_TOOL)

    def set_torque(self, set_data):
        if not isinstance(set_data, int) or set_data not in (0, 1):
            raise ValueError('Torque must be 0 or 1')
        errors = []
        for motor_id in ARM_MOTOR_IDS:
            try:
                result, error = self.packet.write1ByteTxRx(
                    self.port, motor_id, AX_TORQUE_ADDRESS, set_data
                )
                self._check_result(motor_id, 'set torque', result, error)
            except Exception as exc:
                if set_data == 1:
                    raise
                # On explicit torque-off, attempt every motor even if one fails.
                errors.append(f'Motor {motor_id}: {exc}')
        if errors:
            raise RuntimeError('; '.join(errors))

    def set_joint(self, set_data):
        try:
            joint_1_deg, joint_2_deg = set_data
        except (TypeError, ValueError) as exc:
            raise ValueError('Expected exactly two joint angles') from exc

        joint_1_tick = JOINT_1_ZERO_TICK - angle_to_ticks(joint_1_deg, JOINT_MAX_DEG)
        joint_2_tick = JOINT_2_ZERO_TICK + angle_to_ticks(joint_2_deg, JOINT_MAX_DEG)
        # Validate both targets before sending either one.
        validate_position_tick(joint_1_tick)
        validate_position_tick(joint_2_tick)

        for motor_id, tick in ((ARM_JOINT_1, joint_1_tick), (ARM_JOINT_2, joint_2_tick)):
            result, error = self.packet.write2ByteTxRx(
                self.port, motor_id, AX_GOAL_POSITION, tick
            )
            self._check_result(motor_id, 'set joint position', result, error)

    def set_tool(self, set_data):
        tool_tick = TOOL_ZERO_TICK + angle_to_ticks(set_data, TOOL_MAX_DEG, TOOL_MIN_DEG)
        validate_position_tick(tool_tick)
        result, error = self.packet.write2ByteTxRx(
            self.port, ARM_TOOL, AX_GOAL_POSITION, tool_tick
        )
        self._check_result(ARM_TOOL, 'set tool position', result, error)

    def get_arm(self):
        ticks = []
        for motor_id in ARM_MOTOR_IDS:
            tick, result, error = self.packet.read2ByteTxRx(
                self.port, motor_id, AX_PRESENT_POSITION
            )
            self._check_result(motor_id, 'read position', result, error)
            validate_position_tick(tick)
            ticks.append(tick)

        joint_1_tick, joint_2_tick, tool_tick = ticks
        return [
            (JOINT_1_ZERO_TICK - joint_1_tick) * AX_TICK2RAD,
            (joint_2_tick - JOINT_2_ZERO_TICK) * AX_TICK2RAD,
            (tool_tick - TOOL_ZERO_TICK) * AX_TICK2RAD,
        ]
