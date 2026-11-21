#!/usr/bin/env python3

# Python standard library
import math
import time

# ROS 2 core and clocks
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

# ROS 2 message interfaces
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray, Int8, Int32, Int32MultiArray

# Hardware transport and driver interfaces
from dynamixel_sdk import PortHandler
from ai_car.dxl_arm import Arm_Controller
from ai_car.dxl_wheel import Wheel_Controller, VELOCITY_COMMAND_LIMIT, XM_TICK2RAD

# Shared robot geometry and interface constants
from ai_car.constants import (
    ARM_NAMES, JOINT_MIN_DEG, JOINT_MAX_DEG, TOOL_MIN_DEG, TOOL_MAX_DEG,
    WHEEL_NAMES, WHEEL_RADIUS, WHEEL_SEPARATION,
)

# --- Original hardware transport and conversion parameters ---
BAUD_RATE = 1000000
VELOCITY2VALUE = 41.69988758  # Motor velocity units per rad/s.
LIMIT_VEL = VELOCITY_COMMAND_LIMIT  # Shared software limit, not a measured hardware limit.


def constrain(vel):
    """Keep the final motor command inside the existing software limit."""
    return max(-LIMIT_VEL, min(LIMIT_VEL, vel))


def twist_to_wheel_values(linear, angular):
    """Convert a planar body velocity to two signed motor velocity values."""
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError('Twist values must be finite')

    # 1. Differential-drive kinematics: body twist -> wheel linear speeds (m/s).
    left_mps = linear - angular * WHEEL_SEPARATION / 2
    right_mps = linear + angular * WHEEL_SEPARATION / 2

    # 2. Separate physical units from the device representation (rad/s -> raw units).
    left_rad_s = left_mps / WHEEL_RADIUS
    right_rad_s = right_mps / WHEEL_RADIUS
    left_value = left_rad_s * VELOCITY2VALUE
    right_value = right_rad_s * VELOCITY2VALUE
    if not math.isfinite(left_value) or not math.isfinite(right_value):
        raise ValueError('Wheel velocity conversion overflowed')

    # 3. Scale both sides together if needed, preserving the requested curvature.
    peak = max(abs(left_value), abs(right_value))
    if peak > LIMIT_VEL:
        scale = LIMIT_VEL / peak
        left_value *= scale
        right_value *= scale
    return [int(constrain(left_value)), int(constrain(right_value))]


def encoder_delta(current, previous):
    """Find a signed tick difference across a 32-bit rollover."""
    # This assumes less than half the counter range between samples; a hardware
    # reboot or position reset is a separate discontinuity, not a normal rollover.
    delta = (current - previous + 2**31) % 2**32 - 2**31
    if delta == -2**31:
        raise ValueError('Ambiguous encoder rollover')
    return delta


class AICAR_BASE(Node):
    def __init__(self):
        super().__init__('aicar_base')

        # Initialize cleanup state first so a partial startup can be handled.
        self.dxl_port = None
        self.arm = None
        self.wheel = None
        self.timer = None
        self.closed = False
        self.faulted = False
        # These are software command permissions, not measured torque states.
        self.arm_enabled = False
        self.wheel_enabled = False

        # --- Command state ---
        # Callbacks keep the latest valid target; the timer alone performs I/O.
        self.torque_data = 0
        self.wheel_torque_data = 0
        self.tool_data = 0
        self.joint_data = [0, 0]
        self.cmd_data = [0, 0]
        self.reset_received = False
        self.torque_received = False
        self.wheel_torque_received = False
        self.tool_received = False
        self.joint_received = False
        self.cmd_received = False
        self.wheel_stop_requested = False
        self.last_cmd_time = None

        # --- Encoder state (LF, RF, LR, RR) ---
        # None means the first valid read must establish a new baseline.
        self.last_wheel_ticks = None
        self.diff_wheel_rad = [0.0] * 4
        self.total_wheel_rad = [0.0] * 4
        self.arm_states = []

        try:
            # A timeout limits stale commands; validate it before opening the port.
            self.command_timeout = float(
                self.declare_parameter('command_timeout_sec', 0.5).value
            )
            if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
                raise ValueError('command_timeout_sec must be finite and positive')

            # --- Initialize the DYNAMIXEL port ---
            # Teacher steps: create PortHandler, open port, set baud rate, log success.
            # SDK methods return status values; a lack of exceptions is not proof of success.
            self.get_logger().info('Connect U2D2 ...')
            self.dxl_port = PortHandler('/dev/U2D2')
            if not self.dxl_port.openPort():
                raise RuntimeError('Cannot open /dev/U2D2')
            if not self.dxl_port.setBaudRate(BAUD_RATE):
                raise RuntimeError('Cannot configure DYNAMIXEL baud rate')

            # Teacher step: construct both controllers using one shared port.
            # Verify and request a wheel stop first; creating a controller must
            # not authorize motion or automatically home the arm.
            self.wheel = Wheel_Controller(self.dxl_port)
            self.arm = Arm_Controller(self.dxl_port)

            # --- Subscribers ---
            # Teacher steps: register the original five interfaces and callbacks.
            # Preserve /torque as arm-only; a separate request enables the wheels.
            self.reset_sub = self.create_subscription(Empty, '/reset', self.cb_reset, 10)
            self.torque_sub = self.create_subscription(Int8, '/torque', self.cb_torque, 10)
            self.tool_sub = self.create_subscription(Int32, '/tool', self.cb_tool, 1)
            self.joint_sub = self.create_subscription(Int32MultiArray, '/joint', self.cb_joint, 1)
            self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 1)
            self.wheel_torque_sub = self.create_subscription(
                Int8, '/wheel_torque', self.cb_wheel_torque, 10
            )

            # --- Publishers ---
            # Teacher steps: publish JointState and left/right angular increments.
            self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 1)
            self.diff_rad_pub = self.create_publisher(Float32MultiArray, '/diff_rad', 1)

            # --- JointState initialization ---
            # Keep the names and ordering consistent with the URDF and driver feedback.
            self.joint_states = JointState()
            self.joint_states.name = [
                *WHEEL_NAMES,
                *ARM_NAMES,
            ]

            # --- Main control timer (30 Hz) ---
            # Teacher step: periodically read feedback and handle queued commands.
            # Use steady time for watchdog scheduling, so ROS-time jumps cannot
            # extend freshness. Message stamps still use the ROS clock.
            self.timer = self.create_timer(
                1.0 / 30.0, self.cb_timer, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
            self.get_logger().info('[ OK ] AICAR start; motion permissions are disabled')
        except BaseException:
            # Construction may fail after opening the port or one controller succeeds.
            self.fn_shutdown()
            self._try_cleanup('destroy node', self.destroy_node)
            raise

    # --- Subscriber callbacks ---

    def cb_reset(self, msg):
        """Teacher steps: log reset, clear odometry baseline, set reset flag."""
        if self.closed or self.faulted:
            return
        self.get_logger().info('[ SET ] AICAR reset')
        # A reset invalidates pending motion instead of replaying pre-reset targets.
        self._clear_motion_commands()
        self.reset_received = True

    def cb_torque(self, msg):
        """Teacher steps: read, log and store the arm torque request."""
        if self.closed or self.faulted:
            return
        if msg.data not in (0, 1):
            self.get_logger().warning('Invalid arm torque request')
            return
        # A pending disable takes priority over a later enable in the same cycle.
        if self.torque_received and self.torque_data == 0 and msg.data == 1:
            return
        self.get_logger().info(f'[ SET ] ARM Torque : {msg.data}')
        self.torque_data = msg.data
        self.torque_received = True
        self.tool_received = self.joint_received = False

    def cb_wheel_torque(self, msg):
        """Record explicit wheel activation separately from the arm interface."""
        if self.closed or self.faulted:
            return
        if msg.data not in (0, 1):
            self.get_logger().warning('Invalid wheel torque request')
            return
        if self.wheel_torque_received and self.wheel_torque_data == 0 and msg.data == 1:
            return
        self.wheel_torque_data = msg.data
        self.wheel_torque_received = True
        self._clear_wheel_command()

    def cb_tool(self, msg):
        """Teacher steps: validate, log and store the tool target and pending flag."""
        if self.closed or self.faulted or not self.arm_enabled or self.reset_received:
            return
        if not TOOL_MIN_DEG <= msg.data <= TOOL_MAX_DEG:
            self.get_logger().warning('Invalid tool angle')
            return
        if self.torque_received:
            return
        # An absolute setpoint uses latest-wins, not a queue of older positions.
        self.get_logger().info(f'[ SET ] ARM Tool : {msg.data:.2f}')
        self.tool_data = msg.data
        self.tool_received = True

    def cb_joint(self, msg):
        """Teacher steps: validate, log and store two joint targets and pending flag."""
        if self.closed or self.faulted or not self.arm_enabled or self.reset_received:
            return
        if len(msg.data) != 2 or not all(JOINT_MIN_DEG <= angle <= JOINT_MAX_DEG for angle in msg.data):
            self.get_logger().warning('Expected two valid joint angles')
            return
        if self.torque_received:
            return
        self.get_logger().info(f'[ SET ] ARM Joint : {msg.data[0]:.2f}, {msg.data[1]:.2f}')
        self.joint_data = list(msg.data)
        self.joint_received = True

    def cb_cmd(self, msg):
        """Teacher steps: read Twist, convert wheel speeds, limit, log and store."""
        if (self.closed or self.faulted or not self.wheel_enabled or self.reset_received
                or self.wheel_torque_received or self.wheel_stop_requested):
            return
        # Only valid, fresh commands can be executed. A malformed command clears
        # any existing target rather than leaving an old motion request active.
        try:
            values = twist_to_wheel_values(msg.linear.x, msg.angular.z)
        except ValueError as exc:
            self.get_logger().warning(f'Invalid /cmd_vel: {exc}')
            self._clear_wheel_command()
            self.wheel_stop_requested = True
            return

        if values == [0, 0]:
            # A stop request takes priority until the next control cycle handles it.
            self._clear_wheel_command()
            self.wheel_stop_requested = True
            return
        self.get_logger().debug(f'[ SET ] WHEEL : {values[0]}, {values[1]}')
        self.cmd_data = values
        self.cmd_received = True
        # Freshness means local reception time; Twist itself has no timestamp.
        self.last_cmd_time = time.monotonic()

    # --- Main timer callback ---

    def cb_timer(self):
        """
        Teacher tasks: update states; publish joint states and diff_rad; process
        tool, joint, wheel, reset and torque requests. Safety-related requests
        are now handled first so stale or pending movement cannot outrank a stop.
        """
        if self.closed or self.faulted:
            return
        try:
            # 1. Reset the software baseline and request a wheel stop.
            # The original teacher reset changed motor modes, homed the arm and
            # used a 2-second cb_flag timer. A valid encoder sample is the new
            # baseline; shared odometry reset must not unexpectedly move the arm.
            if self.reset_received:
                self._clear_motion_commands()
                self.wheel.reset()
                self.last_wheel_ticks = None
                self.diff_wheel_rad = [0.0] * 4
                # Keep cumulative JointState continuous across a shared reset.
                # Odometry owns its pose origin; resetting both independently
                # could otherwise create a false displacement between callbacks.
                self.reset_received = False
                if self.torque_data == 1:
                    self.torque_received = False
                if self.wheel_torque_data == 1:
                    self.wheel_torque_received = False

            # 2. Process explicit torque-off requests before any ordinary motion.
            if self.wheel_torque_received and self.wheel_torque_data == 0:
                self.wheel_enabled = False
                self._clear_wheel_command()
                self.wheel.reset()
                self.wheel.set_torque(0)
                self.wheel_torque_received = False
            # 3. Check command age even when no subscriber callback arrives.
            # A ROS timer is not an independent hardware safety watchdog.
            if (self.last_cmd_time is not None
                    and time.monotonic() - self.last_cmd_time >= self.command_timeout):
                self.get_logger().warning('Wheel command expired; requesting stop')
                self.wheel_stop_requested = True
            if self.wheel_stop_requested:
                self._clear_wheel_command()
                self.wheel.reset()

            # Wheel stop cannot wait behind unrelated arm bus operations.
            if self.torque_received and self.torque_data == 0:
                self.arm_enabled = False
                self.tool_received = self.joint_received = False
                self.arm.set_torque(0)
                self.torque_received = False

            # 4. Process explicit enable requests without replaying stored commands.
            # The operator must verify physical safety; these are ordinary ROS topics.
            if self.wheel_torque_received:
                self._clear_wheel_command()
                self.wheel.reset()
                self.wheel.set_torque(1)
                self.wheel_enabled = True
                self.wheel_torque_received = False
            if self.torque_received:
                self.tool_received = self.joint_received = False
                self.arm.set_torque(1)
                self.arm_enabled = True
                self.torque_received = False

            # 5. Process the latest valid tool, joint and wheel targets once.
            if self.arm_enabled and self.tool_received:
                self.arm.set_tool(self.tool_data)
                self.tool_received = False
            if self.arm_enabled and self.joint_received:
                self.arm.set_joint(self.joint_data)
                self.joint_received = False
            if self.wheel_enabled and self.cmd_received:
                # The preceding arm I/O may have consumed time; recheck freshness.
                if time.monotonic() - self.last_cmd_time >= self.command_timeout:
                    self._clear_wheel_command()
                    self.wheel.reset()
                else:
                    self.wheel.set_wheel(self.cmd_data)
                    self.cmd_received = False

            # 6. Read and validate actual feedback, not the commanded values.
            # The motor firmware controls the actuator; these reads update ROS state.
            self.fn_update_states()

            # 7. Publish JointState with the original seven positions, in radians.
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            self.joint_states.position = self.total_wheel_rad + self.arm_states
            self.joint_states_pub.publish(self.joint_states)

            # 8. Publish the existing front-left/right angular-increment contract.
            # Retain the original teaching topic for compatibility; odometry
            # uses the stamped cumulative wheel positions in JointState.
            diff_msg = Float32MultiArray()
            diff_msg.data = self.diff_wheel_rad[:2]
            self.diff_rad_pub.publish(diff_msg)
        except Exception as exc:
            # A failed operation must not publish fake feedback or replay commands.
            # Latch the fault; do not automatically resume when communication returns.
            self.faulted = True
            self.arm_enabled = self.wheel_enabled = False
            self._clear_motion_commands()
            self.get_logger().error(f'Hardware/control fault: {exc}')
            self._try_stop_wheels()
            self.get_logger().warning('Arm motion/torque state is unverified; secure hardware manually')

    # --- Feedback and lifecycle helpers ---

    def fn_update_states(self):
        """
        Teacher steps: read arm and wheel feedback, initialize last ticks on the
        first sample, calculate increments, add to total, update last ticks.
        """
        arm = list(self.arm.get_arm())
        wheels = list(self.wheel.get_wheel())
        # Drivers validate communication; Base validates the full data contract
        # before updating any accumulated state or publishing the sample.
        if len(arm) != 3 or not all(math.isfinite(value) for value in arm):
            raise ValueError('Expected three finite arm angles')
        if len(wheels) != 4 or not all(
            type(tick) is int and -(2**31) <= tick < 2**31 for tick in wheels
        ):
            raise ValueError('Expected four signed encoder tick values')

        if self.last_wheel_ticks is None:
            # First valid measurement establishes a reference, not a displacement.
            self.diff_wheel_rad = [0.0] * 4
        else:
            # Preserve genuine small increments; the original 2-tick deadband
            # discarded low-speed travel every time the last tick was updated.
            self.diff_wheel_rad = [
                encoder_delta(current, previous) * XM_TICK2RAD
                for current, previous in zip(wheels, self.last_wheel_ticks)
            ]
            self.total_wheel_rad = [
                total + diff for total, diff in zip(self.total_wheel_rad, self.diff_wheel_rad)
            ]
        self.last_wheel_ticks = wheels
        self.arm_states = [float(value) for value in arm]

    def _clear_wheel_command(self):
        """Discard a speed target so a reset, stop or enable cannot replay it."""
        self.cmd_data = [0, 0]
        self.cmd_received = False
        self.wheel_stop_requested = False
        self.last_cmd_time = None

    def _clear_motion_commands(self):
        """Invalidate pending arm and wheel movement during reset or faults."""
        self._clear_wheel_command()
        self.tool_received = self.joint_received = False

    def _try_cleanup(self, label, action):
        """Attempt each cleanup action independently, even if another fails."""
        try:
            action()
        except Exception as exc:
            self.get_logger().error(f'{label} failed: {exc}')

    def _try_stop_wheels(self):
        """Best-effort requests; communication success is not physical stop proof."""
        if self.wheel is not None:
            self._try_cleanup('wheel stop', self.wheel.reset)
            self._try_cleanup('wheel torque disable', lambda: self.wheel.set_torque(0))

    def fn_shutdown(self):
        """
        Teacher shutdown tasks: log, stop wheels, handle torque and close the port.
        Do not run the original automatic arm home or arbitrary 3-second sleep:
        new motion during shutdown and elapsed time are not a verified safe state.
        """
        if self.closed:
            return
        self.closed = True
        self.arm_enabled = self.wheel_enabled = False
        self._clear_motion_commands()
        self.get_logger().info('Exiting: requesting wheel stop and closing the port')
        if self.timer is not None:
            self._try_cleanup('cancel timer', self.timer.cancel)
        self._try_stop_wheels()
        # Do not automatically remove arm holding torque: an unsupported arm
        # can fall. Its physical state must be secured before power removal.
        if self.arm is not None:
            self.get_logger().warning('Arm torque unchanged; secure and verify arm state manually')
        if self.dxl_port is not None:
            self._try_cleanup('close port', self.dxl_port.closePort)
        self.get_logger().info('Shutdown cleanup attempted')


def main(args=None):
    """Initialize, spin and clean up the node even after partial startup failures."""
    rclpy.init(args=args)
    node = None
    try:
        # Construction is inside try so ROS context cleanup still runs on failure.
        node = AICAR_BASE()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.fn_shutdown()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
