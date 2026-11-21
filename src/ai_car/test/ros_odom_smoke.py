"""Exercise real ROS 2 Odometry and TF messages using synthetic measurements."""

import math
from unittest.mock import Mock, patch

import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from ai_car import aicar_odom


def main():
    rclpy.init()
    node = None
    try:
        node = aicar_odom.AICAR_ODOM()
        clock = Mock()
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = 2.0
        wheel = JointState()
        wheel.name = list(aicar_odom.WHEEL_NAMES[:2])
        with patch.object(node, 'get_clock', return_value=clock), \
                patch.object(node.broadcaster, 'sendTransform', wraps=node.broadcaster.sendTransform) as tf:
            for ns, angles in ((10_000_000_000, [0.0, 0.0]),
                               (10_050_000_000, [0.5, 1.5])):
                clock.now.return_value = Time(nanoseconds=ns)
                imu.header.stamp = clock.now.return_value.to_msg()
                wheel.header.stamp = imu.header.stamp
                wheel.position = angles
                node.cb_imu(imu)
                node.cb_calc_odom(wheel)
            tf.assert_called_once()
            transform = tf.call_args.args[0]
            assert math.isclose(node.x, 0.033 * math.cos(0.05))
            assert math.isclose(node.theta, 0.1)
            assert transform.header.stamp == node.odom.header.stamp
            assert transform.transform.rotation == node.odom.pose.pose.orientation
            assert transform.child_frame_id == 'base_footprint'
            print('ROS 2 Odometry and TF smoke test passed (synthetic data)')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
