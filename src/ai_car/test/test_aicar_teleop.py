"""Offline keyboard/ROS interface tests; never open a terminal or motor device."""

import math
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    def __init__(self, name):
        self.clock = Mock()
        self.logger = Mock()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def create_subscription(self, *args):
        return Mock()

    def create_publisher(self, *args):
        return Mock()

    def destroy_node(self):
        pass


def fake_module(name, **attrs):
    module = ModuleType(name)
    module.__dict__.update(attrs)
    return module


def twist():
    return NS(linear=NS(x=0.0), angular=NS(z=0.0))


with patch.dict(sys.modules, {
    'rclpy': fake_module('rclpy', init=Mock(), ok=Mock(return_value=True), shutdown=Mock()),
    'rclpy.node': fake_module('rclpy.node', Node=FakeNode),
    'rclpy.executors': fake_module('rclpy.executors', SingleThreadedExecutor=Mock()),
    'std_msgs': fake_module('std_msgs'),
    'std_msgs.msg': fake_module('std_msgs.msg', Empty=NS, Int8=NS, Int32=NS, Int32MultiArray=NS),
    'geometry_msgs': fake_module('geometry_msgs'),
    'geometry_msgs.msg': fake_module('geometry_msgs.msg', Twist=twist),
    'sensor_msgs': fake_module('sensor_msgs'), 'sensor_msgs.msg': fake_module('sensor_msgs.msg', JointState=NS),
}):
    from ai_car import aicar_teleop


class TestTeleop(unittest.TestCase):
    def setUp(self):
        self.node = aicar_teleop.AICAR_TELEOP()
        self.now_ns = 10_000_000_000
        self.now_mono = 10.0
        self.node.clock.now.side_effect = lambda: NS(nanoseconds=self.now_ns)
        patcher = patch.object(aicar_teleop.time, 'monotonic', side_effect=lambda: self.now_mono)
        patcher.start()
        self.addCleanup(patcher.stop)

    def advance(self, seconds):
        self.now_ns += round(seconds * 1e9)
        self.now_mono += seconds

    def feedback(self, values=(60, 30, 20), names=aicar_teleop.ARM_NAMES, age=0.0):
        sec, nanosec = divmod(self.now_ns - round(age * 1e9), 1_000_000_000)
        self.node.cb_joint_states(NS(name=list(names), position=[math.radians(x) for x in values],
                                      header=NS(stamp=NS(sec=sec, nanosec=nanosec))))

    def last_wheel(self):
        cmd = self.node.cmd_pub.publish.call_args.args[0]
        return cmd.linear.x, cmd.angular.z

    def test_gradual_wheel_steps_and_limits(self):
        self.node._handle_key('w')
        self.assertEqual(self.last_wheel(), (0.01, 0.0))
        for _ in range(30):
            self.node._handle_key('w')
            self.node._handle_key('a')
        self.assertEqual(self.last_wheel(), (0.15, 4.5))

    def test_lease_refresh_only_until_input_expires(self):
        self.node._handle_key('w')
        self.advance(0.1)
        self.node._handle_key('')
        self.assertEqual(self.last_wheel(), (0.01, 0.0))
        self.advance(0.25)
        self.node._handle_key('')
        self.assertEqual(self.last_wheel(), (0.0, 0.0))
        count = self.node.cmd_pub.publish.call_count
        self.node._handle_key('')
        self.assertEqual(self.node.cmd_pub.publish.call_count, count)
        self.node._handle_key('w')
        self.assertEqual(self.last_wheel(), (0.01, 0.0))

    def test_stop_and_torque_requests_clear_lease(self):
        self.node._handle_key('w')
        self.node._handle_key('s')
        self.assertEqual(self.last_wheel(), (0.0, 0.0))
        self.node._handle_key('w')
        self.node.cb_wheel_torque(NS(data=0))
        count = self.node.cmd_pub.publish.call_count
        self.node._handle_key('')
        self.assertEqual(self.node.cmd_pub.publish.call_count, count)
        self.assertIsNone(self.node.last_wheel_key)

    def test_arm_commands_require_fresh_feedback(self):
        for key in ('u', 'o', 'h'):
            self.node._handle_key(key)
        self.node.joint_pub.publish.assert_not_called()
        self.node.tool_pub.publish.assert_not_called()
        self.feedback()
        self.advance(0.6)
        self.node._handle_key('u')
        self.node.joint_pub.publish.assert_not_called()

    def test_named_feedback_and_target_not_overwritten(self):
        self.feedback(values=(20, 60, 30), names=('tool_joint', 'arm_joint_1', 'arm_joint_2'))
        self.node._handle_key('u')
        self.assertEqual(self.node.joint_pub.publish.call_args.args[0].data, [62, 30])
        self.node.tool_pub.publish.assert_not_called()
        self.advance(0.05)
        self.feedback((61, 30, 20))
        self.node._handle_key('u')
        self.assertEqual(self.node.joint_pub.publish.call_args.args[0].data, [64, 30])

    def test_tool_only_and_explicit_home(self):
        self.feedback()
        self.node._handle_key('o')
        self.assertEqual(self.node.tool_pub.publish.call_args.args[0].data, 22)
        self.node.joint_pub.publish.assert_not_called()
        self.node._handle_key('h')
        self.assertEqual(self.node.tool_pub.publish.call_args.args[0].data, 0)
        self.assertEqual(self.node.joint_pub.publish.call_args.args[0].data, [0, 0])

    def test_reset_does_not_home_arm_and_requires_new_feedback(self):
        self.feedback()
        self.node._handle_key('w')
        self.node.cb_reset(NS())
        self.assertEqual(self.last_wheel(), (0.0, 0.0))
        self.node._handle_key('u')
        self.node.joint_pub.publish.assert_not_called()
        self.advance(0.1)
        self.feedback(age=0.15)
        self.assertFalse(self.node.get_arm_data)
        self.feedback((40, 20, 10))
        self.node._handle_key('u')
        self.assertEqual(self.node.joint_pub.publish.call_args.args[0].data, [42, 20])

    def test_invalid_feedback_and_torque_transition(self):
        self.feedback(values=(math.nan, 30, 20))
        self.feedback(values=(60, 30, 50))
        self.feedback(names=('arm_joint_1', 'arm_joint_1', 'tool_joint'))
        self.assertFalse(self.node.get_arm_data)
        self.feedback()
        self.node.cb_arm_torque(NS(data=1))
        self.node._handle_key('u')
        self.node.joint_pub.publish.assert_not_called()
        self.advance(0.1)
        self.feedback((50, 30, 20))
        self.node._handle_key('u')
        self.assertEqual(self.node.joint_pub.publish.call_args.args[0].data, [52, 30])

    def test_arm_operation_does_not_extend_wheel_lease(self):
        self.feedback()
        self.node._handle_key('w')
        self.advance(0.15)
        self.node._handle_key('u')
        self.advance(0.2)
        self.node._handle_key('o')
        self.assertEqual(self.last_wheel(), (0.0, 0.0))

    def test_no_automatic_enable_and_arrow_byte_ignored(self):
        self.node._handle_key('A')
        self.node.cmd_pub.publish.assert_not_called()
        self.assertIsNone(self.node.last_wheel_key)

    def test_terminal_eof_stops_and_restores_settings(self):
        self.node._handle_key('w')
        with patch.object(aicar_teleop.sys, 'stdin', Mock()), \
                patch.object(aicar_teleop.termios, 'tcgetattr', return_value=['mock settings']), \
                patch.object(aicar_teleop.termios, 'tcsetattr') as restore, \
                patch.object(self.node, 'fn_get_key', return_value=None), patch('builtins.print'):
            self.node.fn_run()
        self.assertEqual(self.last_wheel(), (0.0, 0.0))
        restore.assert_called_once()


if __name__ == '__main__':
    unittest.main()
