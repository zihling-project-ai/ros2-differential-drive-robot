"""Shared AICAR geometry, command limits and ROS interface identifiers."""

# Robot geometry (meters). Keep control and odometry on the same model.
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.16

# Original arm command limits (degrees), not verified mechanical limits.
JOINT_MIN_DEG = 0
JOINT_MAX_DEG = 90
TOOL_MIN_DEG = 0
TOOL_MAX_DEG = 45

# Explicit home pose (degrees); a reset does not request this pose.
HOME_POS = (0, 0)
HOME_TOOL = 0

# Shared identifiers must match the existing robot description.
ARM_NAMES = ('arm_joint_1', 'arm_joint_2', 'tool_joint')
IMU_FRAME_ID = 'imu_link'
