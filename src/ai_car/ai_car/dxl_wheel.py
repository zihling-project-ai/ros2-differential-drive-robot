#!/usr/bin/env python3
import time
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/U2D2')

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

def to_int32(value):
    if value > 0x7FFFFFFF: return value - 0x100000000
    return value

def to_uint32(value):
    return int(value) & 0xFFFFFFFF

class Wheel_Controller():
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
        set_data = to_uint32(set_data)
        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_TORQUE_ADDRESS, set_data)

    def set_wheel(self, set_data):
        set_data = [to_uint32(x) for x in set_data]
        self.packet.write4ByteTxRx(self.port, LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])
        self.packet.write4ByteTxRx(self.port, REAR_LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, REAR_RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])

    def get_wheel(self):
        get_data = [0,0,0,0]
        get_data[0], result, error = self.packet.read4ByteTxRx(self.port, LEFT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[1], result, error = self.packet.read4ByteTxRx(self.port, RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[2], result, error = self.packet.read4ByteTxRx(self.port, REAR_LEFT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[3], result, error = self.packet.read4ByteTxRx(self.port, REAR_RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)
        
        # get_data = np.int32(get_data).tolist()
        # get_data = np.array(get_data, dtype=np.uint32).view(np.int32).tolist()
        get_data = [to_int32(x) for x in get_data]

        return get_data
    
if __name__ == '__main__':

    print('dynamixel wheel controller')
    try:
        DXL_PORT.openPort()
        DXL_PORT.setBaudRate(BAUD_RATE)

        wheel = Wheel_Controller(DXL_PORT)
        
        while True:
            wheel.set_wheel([20, -20])
            time.sleep(1)
    except KeyboardInterrupt:
        wheel.set_wheel([0,0])
