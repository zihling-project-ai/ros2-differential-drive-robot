"""Exercise the real ROS 2 Node API using fake drivers in the isolated CI image."""

import sys
from types import ModuleType
from unittest.mock import Mock, patch

import rclpy

# Do not import the real SDK or open any hardware device.
fake_sdk = ModuleType('dynamixel_sdk')
fake_sdk.COMM_SUCCESS = 0
fake_sdk.PacketHandler = Mock()
fake_sdk.PortHandler = Mock()
with patch.dict(sys.modules, {'dynamixel_sdk': fake_sdk}):
    from ai_car import aicar_base


def main():
    port = Mock()
    port.openPort.return_value = True
    port.setBaudRate.return_value = True
    arm = Mock()
    arm.get_arm.return_value = [0.0, 0.0, 0.0]
    wheel = Mock()
    wheel.get_wheel.return_value = [0, 0, 0, 0]

    rclpy.init()
    node = None
    try:
        with patch.object(aicar_base, 'PortHandler', return_value=port), \
                patch.object(aicar_base, 'Arm_Controller', return_value=arm), \
                patch.object(aicar_base, 'Wheel_Controller', return_value=wheel):
            node = aicar_base.AICAR_BASE()
            node.cb_timer()
            assert not node.faulted
            assert not node.arm_enabled and not node.wheel_enabled
            assert len(node.joint_states.position) == 7
            wheel.set_wheel.assert_not_called()
            arm.set_joint.assert_not_called()
            arm.set_torque.assert_not_called()
            wheel.set_torque.assert_not_called()
            print('ROS 2 Base timer, message and publisher smoke test passed (fake hardware)')
    finally:
        if node is not None:
            node.fn_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
