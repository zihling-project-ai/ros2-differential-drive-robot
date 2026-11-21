"""Offline Base tests: replace ROS and motor drivers with controlled fakes."""

import math
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.logger = Mock()
        self.destroyed = False

    def get_logger(self):
        return self.logger

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=default)

    def create_subscription(self, *args):
        return Mock()

    def create_publisher(self, *args):
        return Mock()

    def create_timer(self, *args, **kwargs):
        return Mock()

    def get_clock(self):
        return Mock()

    def destroy_node(self):
        self.destroyed = True


class FakeJointState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


def fake_module(name, **attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)
    return module


# Import only against mock ROS messages and an SDK with no serial device access.
fake_modules = {
    'rclpy': fake_module('rclpy'),
    'rclpy.node': fake_module('rclpy.node', Node=FakeNode),
    'rclpy.clock': fake_module('rclpy.clock', Clock=Mock(), ClockType=SimpleNamespace(STEADY_TIME=1)),
    'geometry_msgs': fake_module('geometry_msgs'),
    'geometry_msgs.msg': fake_module('geometry_msgs.msg', Twist=SimpleNamespace),
    'sensor_msgs': fake_module('sensor_msgs'),
    'sensor_msgs.msg': fake_module('sensor_msgs.msg', JointState=FakeJointState),
    'std_msgs': fake_module('std_msgs'),
    'std_msgs.msg': fake_module('std_msgs.msg', Empty=SimpleNamespace, Int8=SimpleNamespace,
                                Int32=SimpleNamespace, Int32MultiArray=SimpleNamespace,
                                Float32MultiArray=SimpleNamespace),
    'dynamixel_sdk': fake_module('dynamixel_sdk', COMM_SUCCESS=0,
                                 PacketHandler=Mock(), PortHandler=Mock()),
}
with patch.dict(sys.modules, fake_modules):
    from ai_car import aicar_base


def twist(linear=0.0, angular=0.0):
    return SimpleNamespace(linear=SimpleNamespace(x=linear), angular=SimpleNamespace(z=angular))


class TestBase(unittest.TestCase):
    def setUp(self):
        self.port = Mock()
        self.port.openPort.return_value = True
        self.port.setBaudRate.return_value = True
        self.arm = Mock()
        self.arm.get_arm.return_value = [0.0, 0.0, 0.0]
        self.wheel = Mock()
        self.wheel.get_wheel.return_value = [0, 0, 0, 0]
        for name, value in (('PortHandler', self.port), ('Arm_Controller', self.arm),
                            ('Wheel_Controller', self.wheel)):
            patcher = patch.object(aicar_base, name, return_value=value)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)

    def make_base(self):
        return aicar_base.AICAR_BASE()

    def enable_wheels(self, node):
        # Exercise the logical permission through the fake driver only.
        node.cb_wheel_torque(SimpleNamespace(data=1))
        node.cb_timer()
        self.assertTrue(node.wheel_enabled)
        self.wheel.reset_mock()

    def enable_arm(self, node):
        node.cb_torque(SimpleNamespace(data=1))
        node.cb_timer()
        self.assertTrue(node.arm_enabled)
        self.arm.reset_mock()

    def test_kinematics_and_proportional_limit(self):
        self.assertEqual(aicar_base.twist_to_wheel_values(0.0, 0.0), [0, 0])
        left, right = aicar_base.twist_to_wheel_values(0.05, 0.0)
        self.assertEqual(left, right)
        left, right = aicar_base.twist_to_wheel_values(0.0, 0.5)
        self.assertEqual(left, -right)
        # Use a synthetic out-of-range command to check relative scaling offline.
        left, right = aicar_base.twist_to_wheel_values(10.0, 50.0)
        self.assertLessEqual(max(abs(left), abs(right)), aicar_base.LIMIT_VEL)
        self.assertAlmostEqual(left / right, 6 / 14, delta=0.01)
        for invalid in (math.nan, math.inf, -math.inf, 1e308):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                aicar_base.twist_to_wheel_values(invalid, 0.0)

    def test_port_failure_is_not_reported_as_success(self):
        self.port.openPort.return_value = False
        with self.assertRaises(RuntimeError):
            self.make_base()
        self.port.closePort.assert_called_once()
        self.Wheel_Controller.assert_not_called()
        self.Arm_Controller.assert_not_called()

    def test_partial_initialization_cleans_up_connected_wheels(self):
        self.Arm_Controller.side_effect = RuntimeError('mock arm initialization failure')
        with self.assertRaises(RuntimeError):
            self.make_base()
        self.wheel.reset.assert_called_once()
        self.wheel.set_torque.assert_called_once_with(0)
        self.port.closePort.assert_called_once()

    def test_no_automatic_motion_or_replay_before_enable(self):
        node = self.make_base()
        node.cb_cmd(twist(0.05))
        node.cb_joint(SimpleNamespace(data=[10, 20]))
        node.cb_timer()
        self.wheel.set_torque.assert_not_called()
        self.arm.set_torque.assert_not_called()
        self.wheel.set_wheel.assert_not_called()
        self.arm.set_joint.assert_not_called()
        self.enable_wheels(node)
        self.wheel.set_wheel.assert_not_called()

    def test_latest_valid_arm_and_wheel_commands_win(self):
        node = self.make_base()
        self.enable_wheels(node)
        self.enable_arm(node)
        node.cb_tool(SimpleNamespace(data=10))
        node.cb_tool(SimpleNamespace(data=20))
        node.cb_joint(SimpleNamespace(data=[10]))
        node.cb_joint(SimpleNamespace(data=[10, 20]))
        node.cb_joint(SimpleNamespace(data=[20, 30]))
        node.cb_cmd(twist(0.02))
        node.cb_cmd(twist(0.03))
        node.cb_timer()
        self.arm.set_tool.assert_called_once_with(20)
        self.arm.set_joint.assert_called_once_with([20, 30])
        self.wheel.set_wheel.assert_called_once_with(aicar_base.twist_to_wheel_values(0.03, 0.0))

    def test_timeout_requests_stop_without_a_new_callback(self):
        node = self.make_base()
        self.enable_wheels(node)
        with patch.object(aicar_base.time, 'monotonic', return_value=10.0):
            node.cb_cmd(twist(0.03))
            node.cb_timer()
        self.wheel.set_wheel.assert_called_once()
        with patch.object(aicar_base.time, 'monotonic', return_value=11.0):
            node.cb_timer()
            node.cb_timer()
        self.wheel.reset.assert_called_once()
        self.assertIsNone(node.last_cmd_time)
        self.assertFalse(node.cmd_received)

    def test_invalid_twist_requests_stop_before_new_motion(self):
        node = self.make_base()
        self.enable_wheels(node)
        node.cb_cmd(twist(0.05))
        node.cb_cmd(twist(math.nan))
        node.cb_cmd(twist(0.05))
        node.cb_timer()
        self.wheel.reset.assert_called_once()
        self.wheel.set_wheel.assert_not_called()

    def test_reset_invalidates_motion_without_homing_arm(self):
        node = self.make_base()
        self.enable_wheels(node)
        self.enable_arm(node)
        node.cb_cmd(twist(0.03))
        node.cb_joint(SimpleNamespace(data=[10, 20]))
        node.cb_reset(SimpleNamespace())
        node.cb_timer()
        self.wheel.reset.assert_called_once()
        self.wheel.set_wheel.assert_not_called()
        self.arm.set_joint.assert_not_called()
        self.arm.reset.assert_not_called()
        self.assertEqual(node.total_wheel_rad, [0.0] * 4)

    def test_disable_request_cannot_be_overwritten_by_enable(self):
        node = self.make_base()
        self.enable_wheels(node)
        node.cb_wheel_torque(SimpleNamespace(data=0))
        node.cb_wheel_torque(SimpleNamespace(data=1))
        node.cb_cmd(twist(0.05))
        node.cb_timer()
        self.assertFalse(node.wheel_enabled)
        self.wheel.set_torque.assert_called_once_with(0)
        self.wheel.set_wheel.assert_not_called()

    def test_stop_precedes_pending_arm_motion(self):
        node = self.make_base()
        self.enable_wheels(node)
        self.enable_arm(node)
        trace = Mock()
        trace.attach_mock(self.wheel.reset, 'stop')
        trace.attach_mock(self.arm.set_joint, 'joint')
        node.cb_joint(SimpleNamespace(data=[10, 20]))
        node.cb_cmd(twist())
        node.cb_timer()
        self.assertEqual([call[0] for call in trace.mock_calls], ['stop', 'joint'])

    def test_timeout_is_rechecked_after_other_driver_work(self):
        node = self.make_base()
        self.enable_wheels(node)
        with patch.object(aicar_base.time, 'monotonic', return_value=10.0):
            node.cb_cmd(twist(0.03))
        with patch.object(aicar_base.time, 'monotonic', side_effect=[10.1, 10.6]):
            node.cb_timer()
        self.wheel.set_wheel.assert_not_called()
        self.wheel.reset.assert_called_once()

    def test_encoder_baseline_preserves_small_steps_and_rollover(self):
        node = self.make_base()
        self.wheel.get_wheel.return_value = [120, 120, 120, 120]
        node.fn_update_states()
        self.assertEqual(node.total_wheel_rad, [0.0] * 4)
        self.wheel.get_wheel.return_value = [121, 121, 121, 121]
        node.fn_update_states()
        self.assertEqual(node.diff_wheel_rad, [aicar_base.XM_TICK2RAD] * 4)
        self.assertEqual(aicar_base.encoder_delta(-2**31, 2**31 - 1), 1)
        self.assertEqual(aicar_base.encoder_delta(2**31 - 1, -2**31), -1)

    def test_failed_feedback_latches_fault_without_publishing_invalid_state(self):
        node = self.make_base()
        self.arm.get_arm.side_effect = RuntimeError('mock read failure')
        node.cb_timer()
        self.assertTrue(node.faulted)
        self.wheel.reset.assert_called_once()
        self.wheel.set_torque.assert_called_once_with(0)
        node.joint_states_pub.publish.assert_not_called()
        node.diff_rad_pub.publish.assert_not_called()
        node.cb_timer()
        self.assertEqual(self.arm.get_arm.call_count, 1)

    def test_shutdown_attempts_each_cleanup_and_does_not_move_arm(self):
        node = self.make_base()
        self.wheel.reset.side_effect = RuntimeError('mock stop failure')
        node.fn_shutdown()
        node.fn_shutdown()
        self.wheel.set_torque.assert_called_once_with(0)
        self.port.closePort.assert_called_once()
        self.arm.reset.assert_not_called()
        self.arm.set_torque.assert_not_called()
        node.timer.cancel.assert_called_once()


if __name__ == '__main__':
    unittest.main()
