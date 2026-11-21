#!/usr/bin/env python3

# Standard library
from math import sin, cos, isfinite
import time

# ROS2 core and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

# Coordinate transformations
import tf2_ros

# Original hardware geometry (meters)
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.16  # Reserved for wheel-based yaw estimation.
IMU_TIMEOUT_SEC = 0.1  # Provisional; verify on the actual transport.
MAX_ODOM_DT_SEC = 0.2  # Reject gaps instead of integrating stale measurements.


class AICAR_ODOM(Node):
    def __init__(self):
        super().__init__('aicar_odom')

        # --- Wait for a valid IMU measurement ---
        self.get_logger().info('Waiting for /imu ...')
        self.imu_stamp_ns = None
        self.imu_received_at = None
        self.imu_angular_velocity = 0.0
        self.reset_stamp_ns = None
        self.last_clock_ns = None

        # --- Odometry state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # The first valid wheel sample establishes a time baseline only.
        self.last_time = None

        # --- Subscribers: preserve the three original interfaces ---
        self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.create_subscription(Float32MultiArray, '/diff_rad', self.cb_calc_odom, 10)
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)

        # --- Publisher and TF broadcaster ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Odometry message frames ---
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_footprint'

    # --- Subscriber callbacks ---
    def cb_reset(self, msg):
        """Reset the software pose estimate, not the physical robot or IMU."""
        self.get_logger().info('[ SET ] Odom reset')
        now = self._observe_clock()
        self.reset_stamp_ns = now.nanoseconds
        self._clear_imu()
        self.x = self.y = self.theta = 0.0
        self.last_time = None

    def cb_imu(self, msg):
        """Store only the newest valid gyro Z rate and its measurement time."""
        now = self._observe_clock()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (now.nanoseconds - stamp) / 1e9
        if (msg.header.frame_id != 'imu_link' or not isfinite(msg.angular_velocity.z) or
                msg.angular_velocity_covariance[0] == -1.0 or stamp <= 0 or
                not 0.0 <= age <= IMU_TIMEOUT_SEC or
                (self.reset_stamp_ns is not None and stamp <= self.reset_stamp_ns) or
                (self.imu_stamp_ns is not None and stamp <= self.imu_stamp_ns)):
            self.get_logger().warning('Ignoring invalid or out-of-order IMU sample',
                                      throttle_duration_sec=1.0)
            return
        # Do not integrate across an IMU outage that ended between wheel callbacks.
        if not self._imu_is_fresh(now):
            self.last_time = None
        self.imu_angular_velocity = msg.angular_velocity.z
        self.imu_stamp_ns = stamp
        self.imu_received_at = time.monotonic()

    def _clear_imu(self):
        self.imu_stamp_ns = None
        self.imu_received_at = None
        self.imu_angular_velocity = 0.0

    def _observe_clock(self):
        """Discard time baselines after a ROS clock rollback or simulation reset."""
        now = self.get_clock().now()
        if self.last_clock_ns is not None and now.nanoseconds < self.last_clock_ns:
            self.last_time = None
            self.reset_stamp_ns = None
            self._clear_imu()
        self.last_clock_ns = now.nanoseconds
        return now

    def _imu_is_fresh(self, now):
        if self.imu_stamp_ns is None or self.imu_received_at is None:
            return False
        age = (now.nanoseconds - self.imu_stamp_ns) / 1e9
        receive_age = time.monotonic() - self.imu_received_at
        return (0.0 <= age <= IMU_TIMEOUT_SEC and
                0.0 <= receive_age <= IMU_TIMEOUT_SEC)

    def cb_calc_odom(self, msg):
        """Integrate valid wheel increments and gyro rate into a planar pose."""
        now = self._observe_clock()
        # Step 1: Require a fresh IMU sample and exactly two finite wheel increments.
        if len(msg.data) != 2 or not all(isfinite(value) for value in msg.data):
            self.last_time = None
            self.get_logger().warning('Invalid wheel increments', throttle_duration_sec=1.0)
            return
        if not self._imu_is_fresh(now):
            self.last_time = None
            self.get_logger().warning('IMU unavailable or stale; odometry paused',
                                      throttle_duration_sec=1.0)
            return

        # Step 2: Retain the teacher's stationary-wheel bias suppression heuristic.
        # It is not proof of body stationarity (slip or external motion is possible).
        ang_vel = 0.0 if msg.data[0] == msg.data[1] == 0 else self.imu_angular_velocity

        # Step 3: Establish a baseline and reject invalid or excessive time gaps.
        # /diff_rad has no header, so this uses reception time, not acquisition time.
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        if not 0.0 < dt <= MAX_ODOM_DT_SEC:
            self.last_time = now if dt > 0 else None
            self.get_logger().warning('Invalid odometry interval; rebaselining',
                                      throttle_duration_sec=1.0)
            return

        # Step 4: Convert wheel angles to traveled distances (meters).
        left_dist = WHEEL_RADIUS * msg.data[0]
        right_dist = WHEEL_RADIUS * msg.data[1]

        # Step 5: The mean wheel distance approximates the body-center distance.
        distance = (left_dist + right_dist) / 2.0
        lin_vel = distance / dt

        # Step 6: Use the midpoint heading to approximate motion along a curve.
        delta_theta = ang_vel * dt
        theta_mid = self.theta + delta_theta / 2.0
        self.x += distance * cos(theta_mid)
        self.y += distance * sin(theta_mid)
        self.theta += delta_theta

        # Step 7: Fill the estimated pose in odom and twist in base_footprint.
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = sin(self.theta / 2)
        self.odom.pose.pose.orientation.w = cos(self.theta / 2)

        # Planar velocity in the robot's own frame. Covariances are uncalibrated.
        self.odom.twist.twist.linear.x = lin_vel
        self.odom.twist.twist.angular.z = ang_vel
        # Use the same estimate timestamp for odometry and TF.
        self.odom.header.stamp = now.to_msg()
        self.odom_pub.publish(self.odom)

        # Step 8: Broadcast TF (odom -> base_footprint) from the same estimate.
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.odom.header.stamp
        tf_msg.header.frame_id = self.odom.header.frame_id
        tf_msg.child_frame_id = self.odom.child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = self.odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = self.odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = self.odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = self.odom.pose.pose.orientation.w
        self.broadcaster.sendTransform(tf_msg)

        # Step 9: Retain the timestamp for the next integration interval.
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AICAR_ODOM()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # No direct hardware resources; still shut down ROS if creation fails.
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
