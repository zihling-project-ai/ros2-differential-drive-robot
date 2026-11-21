#!/usr/bin/env python3

# Standard library and terminal keyboard input
import math
import select
import sys
import termios
import threading
import time
import tty

# ROS2 core and message types
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Int8, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from ai_car.constants import (
    ARM_NAMES, HOME_POS, HOME_TOOL, JOINT_MIN_DEG, JOINT_MAX_DEG, TOOL_MIN_DEG, TOOL_MAX_DEG,
)

# Original operator velocity limits (m/s, rad/s), not certified safety limits.
MAX_LIN_VEL = 0.15
MAX_ANG_VEL = 4.5

# Restore the teacher's gradual increments instead of jumping to full speed.
STEP_LIN = 0.01
STEP_ANG = 0.5
STEP_ARM = 2

# A short input lease limits stale keyboard targets; Base has its own watchdog.
KEY_TIMEOUT_SEC = 0.3
KEY_POLL_SEC = 0.1
ARM_FEEDBACK_TIMEOUT_SEC = 0.5

msg = """
Control Your AICAR!
----------------------------------------------
Moving around (repeat movement keys to continue):
            w(+)
   a(+)      s      d(-)
            x(-)

Arm control:
    Joint1 : u(+), j(-)
    Joint2 : i(+), k(-)
     Tool  : o(+), l(-)

h : request Arm Home Pose
s : request wheel stop (not a hardware emergency stop)

Motor enable/disable remains a separate explicit operation.
CTRL-C to quit
"""


def constrain(value, maximum, minimum):
    return max(minimum, min(maximum, value))


class AICAR_TELEOP(Node):
    def __init__(self):
        super().__init__('aicar_teleop')

        # --- State: the keyboard and ROS callbacks share these targets ---
        self.lock = threading.Lock()
        self.lin_vel = self.ang_vel = 0.0
        self.last_wheel_key = None
        self.get_arm_data = False
        self.last_arm_received = None
        self.last_arm_stamp_ns = None
        self.arm_cutoff_ns = None
        self.j1 = self.j2 = self.tool = 0

        # --- Subscribers ---
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)
        self.reset_sub = self.create_subscription(Empty, '/reset', self.cb_reset, 10)
        # Observe explicit enable/disable requests only to discard old targets.
        # Receiving a request does not prove that the physical torque has changed.
        self.arm_torque_sub = self.create_subscription(Int8, '/torque', self.cb_arm_torque, 10)
        self.wheel_torque_sub = self.create_subscription(Int8, '/wheel_torque', self.cb_wheel_torque, 10)

        # --- Publishers: retain original command types and topics ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.joint_pub = self.create_publisher(Int32MultiArray, '/joint', 1)
        self.tool_pub = self.create_publisher(Int32, '/tool', 1)

    # --- Subscriber callbacks ---
    def cb_joint_states(self, msg):
        """Initialize targets from fresh named joints, not fixed array indexes."""
        with self.lock:
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            # If ROS time moves backwards, require a new measured baseline.
            if ((self.last_arm_stamp_ns is not None and ros_now < self.last_arm_stamp_ns) or
                    (self.arm_cutoff_ns is not None and ros_now < self.arm_cutoff_ns)):
                self.arm_cutoff_ns = None
                self.last_arm_stamp_ns = None
                self.get_arm_data = False
            if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name) or
                    stamp <= 0 or not 0.0 <= (ros_now - stamp) / 1e9 <= ARM_FEEDBACK_TIMEOUT_SEC or
                    (self.arm_cutoff_ns is not None and stamp <= self.arm_cutoff_ns) or
                    (self.last_arm_stamp_ns is not None and stamp <= self.last_arm_stamp_ns)):
                return
            positions = dict(zip(msg.name, msg.position))
            if not all(name in positions and math.isfinite(positions[name]) for name in ARM_NAMES):
                return
            # Only the initial/recovered sample establishes targets; do not overwrite
            # a user target with the moving arm's instantaneous actual position.
            degrees = [math.degrees(positions[name]) for name in ARM_NAMES]
            if not all(math.isfinite(value) for value in degrees):
                return
            angles = [round(value) for value in degrees]
            if not all(low <= value <= high for value, low, high in
                       zip(angles, (JOINT_MIN_DEG, JOINT_MIN_DEG, TOOL_MIN_DEG),
                           (JOINT_MAX_DEG, JOINT_MAX_DEG, TOOL_MAX_DEG))):
                self.get_arm_data = False
                self.last_arm_received = None
                return
            if not self._arm_feedback_fresh(now):
                self.j1, self.j2, self.tool = angles
                self.get_arm_data = True
                self.get_logger().info(f'[ ARM ] sync: {self.j1}, {self.j2}, {self.tool}')
            self.last_arm_received = now
            self.last_arm_stamp_ns = stamp

    def _arm_feedback_fresh(self, now):
        if not self.get_arm_data or self.last_arm_received is None or self.last_arm_stamp_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.last_arm_stamp_ns) / 1e9
        return (0.0 <= age <= ARM_FEEDBACK_TIMEOUT_SEC and
                0.0 <= now - self.last_arm_received <= ARM_FEEDBACK_TIMEOUT_SEC)

    def _invalidate_arm(self):
        # Reject queued pre-reset/enable feedback; wait for an actual new sample.
        self.get_arm_data = False
        self.last_arm_received = self.last_arm_stamp_ns = None
        self.arm_cutoff_ns = self.get_clock().now().nanoseconds

    def cb_reset(self, msg):
        """Stop wheel targets and request a new arm baseline, not a home pose."""
        with self.lock:
            self._stop_wheels()
            self._invalidate_arm()
            self.get_logger().info('[ SET ] TELEOP reset')

    def cb_arm_torque(self, msg):
        if msg.data in (0, 1):
            with self.lock:
                self._invalidate_arm()

    def cb_wheel_torque(self, msg):
        if msg.data in (0, 1):
            with self.lock:
                # Clear pending targets without publishing a competing command.
                self.last_wheel_key = None
                self.lin_vel = self.ang_vel = 0.0

    # --- Terminal and command publication ---
    def fn_get_key(self):
        """Return one key, an empty timeout, or None on terminal EOF."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], KEY_POLL_SEC)
            if ready:
                return sys.stdin.read(1) or None
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def fn_wheel_pub(self):
        # Publish desired body velocity; Base alone handles motor commands.
        cmd = Twist()
        cmd.linear.x = float(self.lin_vel)
        cmd.angular.z = float(self.ang_vel)
        self.cmd_pub.publish(cmd)

    def fn_arm_pub(self, joint=False, tool=False):
        # Send only the requested group; only an explicit home requests both.
        if joint:
            joint_msg = Int32MultiArray()
            joint_msg.data = [self.j1, self.j2]
            self.joint_pub.publish(joint_msg)
        if tool:
            tool_msg = Int32()
            tool_msg.data = self.tool
            self.tool_pub.publish(tool_msg)

    def _stop_wheels(self):
        self.lin_vel = self.ang_vel = 0.0
        self.last_wheel_key = None
        self.fn_wheel_pub()

    def _handle_key(self, key):
        """Process one input event and refresh only a still-valid wheel target."""
        with self.lock:
            now = time.monotonic()
            # Expire before processing a new key; do not inherit an old target.
            if self.last_wheel_key is not None and now - self.last_wheel_key >= KEY_TIMEOUT_SEC:
                self._stop_wheels()
            if key == 's':
                self._stop_wheels()
                return
            if key in ('w', 'x', 'a', 'd'):
                # Move by the teacher's increments; only wheel keys renew the lease.
                if key == 'w':
                    self.lin_vel = constrain(self.lin_vel + STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                elif key == 'x':
                    self.lin_vel = constrain(self.lin_vel - STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                elif key == 'a':
                    self.ang_vel = constrain(self.ang_vel + STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                else:
                    self.ang_vel = constrain(self.ang_vel - STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                self.last_wheel_key = now
            elif key in ('u', 'j', 'i', 'k', 'o', 'l', 'h'):
                # Require fresh measured feedback before issuing an arm target.
                if not self._arm_feedback_fresh(now):
                    self.get_arm_data = False
                    self.get_logger().warning('Waiting for fresh arm feedback', throttle_duration_sec=1.0)
                elif key == 'h':
                    self.j1, self.j2 = HOME_POS
                    self.tool = HOME_TOOL
                    self.fn_arm_pub(joint=True, tool=True)
                elif key in ('u', 'j'):
                    self.j1 = constrain(self.j1 + (STEP_ARM if key == 'u' else -STEP_ARM),
                                        JOINT_MAX_DEG, JOINT_MIN_DEG)
                    self.fn_arm_pub(joint=True)
                elif key in ('i', 'k'):
                    self.j2 = constrain(self.j2 + (STEP_ARM if key == 'i' else -STEP_ARM),
                                        JOINT_MAX_DEG, JOINT_MIN_DEG)
                    self.fn_arm_pub(joint=True)
                else:
                    self.tool = constrain(self.tool + (STEP_ARM if key == 'o' else -STEP_ARM),
                                          TOOL_MAX_DEG, TOOL_MIN_DEG)
                    self.fn_arm_pub(tool=True)

            # Refresh a bounded user input lease, never a latched old velocity.
            if self.last_wheel_key is not None:
                self.fn_wheel_pub()

    def fn_run(self):
        """Teacher steps: show controls, read keys, publish, stop and restore terminal."""
        if not sys.stdin.isatty():
            raise RuntimeError('Teleop requires an interactive terminal')
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        try:
            while rclpy.ok():
                key = self.fn_get_key()
                if key is None or key == '\x03':
                    break
                # Do not lowercase escape-sequence bytes (e.g. arrow-key 'A').
                self._handle_key(key)
        finally:
            try:
                with self.lock:
                    self._stop_wheels()
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = executor = spin_thread = None
    try:
        node = AICAR_TELEOP()
        # One background executor handles callbacks; the main thread reads keys.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        node.fn_run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if executor is not None:
                executor.shutdown()
            if spin_thread is not None and spin_thread.ident is not None:
                spin_thread.join()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
