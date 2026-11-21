#!/usr/bin/env python3

# Standard library
from math import sin, cos, isfinite
import time

# ROS2 core and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, JointState

# Coordinate transformations
import tf2_ros

# Shared robot geometry and frame identifier
from ai_car.constants import IMU_FRAME_ID, WHEEL_NAMES, WHEEL_RADIUS

IMU_TIMEOUT_SEC = 0.1  # Provisional; verify on the actual transport.
WHEEL_FEEDBACK_TIMEOUT_SEC = 0.2  # Reject delayed wheel measurements.
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
        self.last_wheel_stamp_ns = None
        self.last_wheel_angles = None

        # --- Subscribers: use stamped cumulative wheel feedback ---
        self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.create_subscription(JointState, '/joint_states', self.cb_calc_odom, 10)
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
        self.last_wheel_stamp_ns = None
        self.last_wheel_angles = None

    def cb_imu(self, msg):
        """Store only the newest valid gyro Z rate and its measurement time."""
        now = self._observe_clock()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (now.nanoseconds - stamp) / 1e9
        if (msg.header.frame_id != IMU_FRAME_ID or not isfinite(msg.angular_velocity.z) or
                msg.angular_velocity_covariance[0] == -1.0 or stamp <= 0 or
                not 0.0 <= age <= IMU_TIMEOUT_SEC or
                (self.reset_stamp_ns is not None and stamp <= self.reset_stamp_ns) or
                (self.imu_stamp_ns is not None and stamp <= self.imu_stamp_ns)):
            self.get_logger().warning('Ignoring invalid or out-of-order IMU sample',
                                      throttle_duration_sec=1.0)
            return
        # Do not integrate across an IMU outage that ended between wheel callbacks.
        if not self._imu_is_fresh(now):
            self.last_wheel_angles = None
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
            self.last_wheel_stamp_ns = None
            self.last_wheel_angles = None
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
        """Integrate stamped cumulative wheel angles and gyro rate into a planar pose."""
        now = self._observe_clock()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # Step 1: Reject stale, future, duplicate and pre-reset wheel samples.
        age = (now.nanoseconds - stamp) / 1e9
        if (stamp <= 0 or not 0.0 <= age <= WHEEL_FEEDBACK_TIMEOUT_SEC or
                (self.reset_stamp_ns is not None and stamp <= self.reset_stamp_ns) or
                (self.last_wheel_stamp_ns is not None and stamp <= self.last_wheel_stamp_ns)):
            self.get_logger().warning('Ignoring invalid or out-of-order wheel timestamp',
                                      throttle_duration_sec=1.0)
            return
        # Resolve the front wheel positions by name, not array order.
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.last_wheel_angles = None
            self.get_logger().warning('Invalid JointState structure', throttle_duration_sec=1.0)
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions and isfinite(positions[name]) for name in WHEEL_NAMES[:2]):
            self.last_wheel_angles = None
            self.get_logger().warning('Missing or invalid wheel angles', throttle_duration_sec=1.0)
            return
        angles = tuple(positions[name] for name in WHEEL_NAMES[:2])

        previous_stamp = self.last_wheel_stamp_ns
        previous_angles = self.last_wheel_angles
        self.last_wheel_stamp_ns = stamp
        # The latest gyro rate is an approximation for this wheel interval.
        # Require freshness at reception and bounded skew from the wheel stamp.
        if (not self._imu_is_fresh(now) or
                abs(stamp - self.imu_stamp_ns) / 1e9 > IMU_TIMEOUT_SEC):
            self.last_wheel_angles = None
            self.get_logger().warning('IMU unavailable, stale or misaligned; odometry paused',
                                      throttle_duration_sec=1.0)
            return

        # Step 2: Establish a new baseline after startup, reset or an invalid gap.
        # Cumulative positions let us recover short missing-message intervals.
        self.last_wheel_angles = angles
        if previous_stamp is None or previous_angles is None:
            return
        # Step 3: Use the measurement timestamps, not callback arrival time.
        dt = (stamp - previous_stamp) / 1e9
        if not 0.0 < dt <= MAX_ODOM_DT_SEC:
            self.get_logger().warning('Invalid odometry interval; rebaselining',
                                      throttle_duration_sec=1.0)
            return
        increments = tuple(current - previous for current, previous in zip(angles, previous_angles))
        if not all(isfinite(value) for value in increments):
            self.last_wheel_angles = None
            return

        # Retain the teacher's stationary-wheel bias suppression heuristic.
        # It is not proof of body stationarity (slip or external motion is possible).
        ang_vel = 0.0 if increments[0] == increments[1] == 0 else self.imu_angular_velocity

        # Step 4: Convert wheel angles to traveled distances (meters).
        left_dist = WHEEL_RADIUS * increments[0]
        right_dist = WHEEL_RADIUS * increments[1]

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
        self.odom.header.stamp = msg.header.stamp
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

        # Step 9: The accepted wheel angles and stamp are the next baseline.


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
