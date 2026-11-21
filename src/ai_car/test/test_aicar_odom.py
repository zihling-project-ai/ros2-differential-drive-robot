"""Offline odometry tests with synthetic messages and deterministic clocks."""

import math
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


def vector():
    return NS(x=0.0, y=0.0, z=0.0)


def quaternion():
    return NS(x=0.0, y=0.0, z=0.0, w=1.0)


def header():
    return NS(frame_id='', stamp=None)


def odometry():
    return NS(header=header(), child_frame_id='',
              pose=NS(pose=NS(position=vector(), orientation=quaternion())),
              twist=NS(twist=NS(linear=vector(), angular=vector())))


def transform():
    return NS(header=header(), child_frame_id='',
              transform=NS(translation=vector(), rotation=quaternion()))


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        sec, nanosec = divmod(self.nanoseconds, 1_000_000_000)
        return NS(sec=sec, nanosec=nanosec)


class FakeNode:
    def __init__(self, name):
        self.clock = Mock()
        self.logger = Mock()
        self.publisher = Mock()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def create_subscription(self, *args):
        return Mock()

    def create_publisher(self, *args):
        return self.publisher

    def destroy_node(self):
        pass


def fake_module(name, **attrs):
    module = ModuleType(name)
    module.__dict__.update(attrs)
    return module


with patch.dict(sys.modules, {
    'rclpy': fake_module('rclpy', init=Mock(), spin=Mock(), ok=Mock(return_value=True), shutdown=Mock()),
    'rclpy.node': fake_module('rclpy.node', Node=FakeNode),
    'nav_msgs': fake_module('nav_msgs'), 'nav_msgs.msg': fake_module('nav_msgs.msg', Odometry=odometry),
    'std_msgs': fake_module('std_msgs'), 'std_msgs.msg': fake_module('std_msgs.msg', Empty=NS, Float32MultiArray=NS),
    'sensor_msgs': fake_module('sensor_msgs'), 'sensor_msgs.msg': fake_module('sensor_msgs.msg', Imu=NS, JointState=NS),
    'geometry_msgs': fake_module('geometry_msgs'), 'geometry_msgs.msg': fake_module('geometry_msgs.msg', TransformStamped=transform),
    'tf2_ros': fake_module('tf2_ros', TransformBroadcaster=Mock(side_effect=lambda node: Mock())),
}):
    from ai_car import aicar_odom


class TestOdom(unittest.TestCase):
    def setUp(self):
        self.node = aicar_odom.AICAR_ODOM()
        self.now_ns = 10_000_000_000
        self.receive_time = 10.0
        self.positions = [0.0, 0.0]
        self.node.clock.now.side_effect = lambda: FakeTime(self.now_ns)
        patcher = patch.object(aicar_odom.time, 'monotonic', side_effect=lambda: self.receive_time)
        patcher.start()
        self.addCleanup(patcher.stop)

    def advance(self, seconds):
        self.now_ns += round(seconds * 1e9)
        self.receive_time += seconds

    def imu(self, rate=0.0, age=0.0, frame='imu_link', covariance=0.0):
        stamp = FakeTime(self.now_ns - round(age * 1e9)).to_msg()
        self.node.cb_imu(NS(header=NS(stamp=stamp, frame_id=frame), angular_velocity=NS(z=rate),
                            angular_velocity_covariance=[covariance] + [0.0]*8))

    def wheel_position(self, left, right, age=0.0, names=None):
        stamp = FakeTime(self.now_ns - round(age * 1e9)).to_msg()
        names = list(aicar_odom.WHEEL_NAMES[:2]) if names is None else names
        self.node.cb_calc_odom(NS(header=NS(stamp=stamp), name=names, position=[left, right]))

    def wheel(self, left=0.0, right=0.0, age=0.0):
        # Test helper accepts increments, but the ROS message contains cumulative values.
        self.positions[0] += left
        self.positions[1] += right
        self.wheel_position(*self.positions, age=age)

    def baseline(self):
        self.imu()
        self.wheel()
        self.node.publisher.publish.assert_not_called()

    def test_first_sample_only_establishes_baseline(self):
        self.wheel(1.0, 1.0)
        self.advance(0.01)
        self.imu()
        self.wheel(1.0, 1.0)
        self.assertEqual((self.node.x, self.node.y, self.node.theta), (0, 0, 0))
        self.node.publisher.publish.assert_not_called()

    def test_straight_and_reverse(self):
        self.baseline()
        self.advance(0.05)
        self.imu()
        self.wheel(1.0, 1.0)
        self.assertAlmostEqual(self.node.x, 0.033)
        self.assertAlmostEqual(self.node.odom.twist.twist.linear.x, 0.66)
        self.advance(0.05)
        self.imu()
        self.wheel(-1.0, -1.0)
        self.assertAlmostEqual(self.node.x, 0.0)

    def test_midpoint_turn_and_quaternion_tf_consistency(self):
        self.baseline()
        self.advance(0.05)
        self.imu(2.0)
        self.wheel(0.5, 1.5)
        self.assertAlmostEqual(self.node.x, 0.033 * math.cos(0.05))
        self.assertAlmostEqual(self.node.y, 0.033 * math.sin(0.05))
        self.assertAlmostEqual(self.node.theta, 0.1)
        odom = self.node.odom
        tf = self.node.broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(odom.header.stamp, tf.header.stamp)
        self.assertEqual((tf.header.frame_id, tf.child_frame_id), ('odom', 'base_footprint'))
        self.assertEqual(odom.pose.pose.position, tf.transform.translation)
        self.assertEqual(odom.pose.pose.orientation, tf.transform.rotation)
        q = odom.pose.pose.orientation
        self.assertAlmostEqual(q.z, math.sin(0.05))
        self.assertAlmostEqual(q.z*q.z + q.w*q.w, 1.0)

    def test_in_place_rotation_and_stationary_bias(self):
        self.baseline()
        self.advance(0.05)
        self.imu(1.0)
        self.wheel(-1.0, 1.0)
        self.assertEqual((self.node.x, self.node.y), (0, 0))
        self.assertAlmostEqual(self.node.theta, 0.05)
        self.advance(0.05)
        self.imu(0.1)
        self.wheel()
        self.assertAlmostEqual(self.node.theta, 0.05)

    def test_invalid_or_out_of_order_imu_does_not_replace_valid_data(self):
        self.imu(1.0)
        self.advance(0.02)
        for kwargs in ({'rate': math.nan}, {'rate': math.inf}, {'age': 0.03},
                       {'age': -0.01}, {'frame': 'wrong'}, {'covariance': -1.0}):
            with self.subTest(kwargs=kwargs):
                self.imu(**kwargs)
                self.assertEqual(self.node.imu_angular_velocity, 1.0)
        self.imu(2.0)
        self.assertEqual(self.node.imu_angular_velocity, 2.0)

    def test_expired_imu_pauses_and_rebaselines(self):
        self.baseline()
        self.advance(0.12)
        self.wheel(1.0, 1.0)
        self.assertIsNone(self.node.last_wheel_angles)
        self.node.publisher.publish.assert_not_called()
        self.advance(0.01)
        self.imu(1.0)
        self.wheel(1.0, 1.0)
        self.advance(0.05)
        self.imu(1.0)
        self.wheel(1.0, 1.0)
        self.assertAlmostEqual(self.node.theta, 0.05)

    def test_receive_timeout_even_if_ros_clock_paused(self):
        self.baseline()
        self.receive_time += 0.11
        self.now_ns += 1_000_000  # A new measurement stamp, but old IMU receipt time.
        self.wheel(1.0, 1.0)
        self.assertIsNone(self.node.last_wheel_angles)
        self.node.publisher.publish.assert_not_called()

    def test_imu_outage_between_wheel_callbacks(self):
        self.baseline()
        self.advance(0.15)
        self.imu(1.0)
        self.wheel(1.0, 1.0)
        self.node.publisher.publish.assert_not_called()

    def test_invalid_wheel_message_and_large_dt_drop_interval(self):
        self.baseline()
        self.advance(0.02)
        self.imu()
        names = list(aicar_odom.WHEEL_NAMES[:2])
        for data in ([], [1], [1, 2, 3], [math.nan, 0]):
            self.node.cb_calc_odom(NS(header=NS(stamp=FakeTime(self.now_ns).to_msg()),
                                     name=names, position=data))
        self.assertIsNone(self.node.last_wheel_angles)
        self.wheel()
        for _ in range(5):
            self.advance(0.05)
            self.imu()
        self.wheel(1.0, 1.0)
        self.node.publisher.publish.assert_not_called()

    def test_reset_discards_pre_reset_imu_and_restarts_baseline(self):
        self.baseline()
        self.advance(0.05)
        self.imu()
        self.wheel(1.0, 1.0)
        self.node.cb_reset(NS())
        self.assertEqual((self.node.x, self.node.y, self.node.theta), (0, 0, 0))
        self.advance(0.01)
        self.imu(1.0, age=0.02)
        self.assertIsNone(self.node.imu_stamp_ns)
        self.imu()
        self.wheel()
        self.assertEqual(self.node.publisher.publish.call_count, 1)

    def test_measurement_timestamps_not_arrival_times(self):
        self.imu(age=0.01)
        self.wheel(age=0.01)
        self.advance(0.08)
        self.imu(1.0, age=0.04)
        self.wheel(1.0, 1.0, age=0.04)
        # Actual wheel interval is 0.05 s even though arrival interval is 0.08 s.
        self.assertAlmostEqual(self.node.odom.twist.twist.linear.x, 0.033 / 0.05)
        self.assertAlmostEqual(self.node.theta, 0.05)
        self.assertEqual(self.node.odom.header.stamp, FakeTime(self.now_ns - 40_000_000).to_msg())

    def test_short_missing_wheel_message_recovers_cumulative_distance(self):
        self.baseline()
        self.advance(0.04)
        self.imu()
        # No wheel callback for this interval; the next sample spans both.
        self.advance(0.04)
        self.imu()
        self.wheel(2.0, 2.0)
        self.assertAlmostEqual(self.node.x, 0.066)
        self.assertAlmostEqual(self.node.odom.twist.twist.linear.x, 0.066 / 0.08)

    def test_duplicate_and_out_of_order_wheel_samples_do_not_replace_baseline(self):
        self.baseline()
        self.advance(0.05)
        self.imu()
        self.wheel_position(1.0, 1.0)
        self.wheel_position(100.0, 100.0)  # Duplicate timestamp.
        self.wheel_position(100.0, 100.0, age=0.01)  # Out of order.
        self.advance(0.05)
        self.imu()
        self.wheel_position(2.0, 2.0)
        self.assertAlmostEqual(self.node.x, 0.066)
        self.assertEqual(self.node.publisher.publish.call_count, 2)

    def test_wheel_names_not_array_order(self):
        self.baseline()
        self.advance(0.05)
        self.imu(2.0)
        self.wheel_position(1.5, 0.5, names=list(reversed(aicar_odom.WHEEL_NAMES[:2])))
        self.assertAlmostEqual(self.node.x, 0.033 * math.cos(0.05))

    def test_reset_rejects_queued_wheel_and_preserves_continuous_angles(self):
        self.imu()
        self.wheel_position(10.0, 10.0)
        self.advance(0.05)
        self.imu()
        self.wheel_position(11.0, 11.0)
        self.node.cb_reset(NS())
        self.advance(0.01)
        self.imu()
        self.wheel_position(11.0, 11.0, age=0.02)  # Queued pre-reset feedback.
        self.assertIsNone(self.node.last_wheel_angles)
        self.wheel_position(11.0, 11.0)  # Odom reset before Base's timer.
        self.advance(0.05)
        self.imu()
        self.wheel_position(11.0, 11.0)  # Base rebaselines but keeps cumulative angles.
        self.assertEqual((self.node.x, self.node.y, self.node.theta), (0, 0, 0))
        self.advance(0.05)
        self.imu()
        self.wheel_position(12.0, 12.0)
        self.assertAlmostEqual(self.node.x, 0.033)

    def test_base_reset_feedback_before_odom_reset(self):
        self.imu()
        self.wheel_position(10.0, 10.0)
        self.advance(0.05)
        self.imu()
        self.wheel_position(10.0, 10.0)  # Base has reset its raw tick baseline.
        self.node.cb_reset(NS())
        self.advance(0.05)
        self.imu()
        self.wheel_position(10.0, 10.0)
        self.advance(0.05)
        self.imu()
        self.wheel_position(11.0, 11.0)
        self.assertAlmostEqual(self.node.x, 0.033)

    def test_future_old_and_misaligned_wheel_samples_are_not_integrated(self):
        self.baseline()
        self.advance(0.15)
        self.imu()
        self.wheel_position(20.0, 20.0, age=-0.01)
        self.wheel_position(20.0, 20.0, age=0.3)
        # Valid wheel stamp but too far from latest IMU time.
        self.wheel_position(1.0, 1.0, age=0.11)
        self.node.publisher.publish.assert_not_called()
        self.assertIsNone(self.node.last_wheel_angles)

    def test_ros_clock_rollback_restarts_baseline(self):
        self.baseline()
        self.now_ns -= 1_000_000_000
        self.imu()
        self.wheel(1.0, 1.0)
        self.node.publisher.publish.assert_not_called()
        self.advance(0.05)
        self.imu()
        self.wheel(1.0, 1.0)
        self.assertAlmostEqual(self.node.x, 0.033)

    def test_main_shuts_down_ros_on_constructor_failure(self):
        aicar_odom.rclpy.shutdown.reset_mock()
        with patch.object(aicar_odom, 'AICAR_ODOM', side_effect=RuntimeError('mock init failure')):
            with self.assertRaises(RuntimeError):
                aicar_odom.main()
        aicar_odom.rclpy.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
