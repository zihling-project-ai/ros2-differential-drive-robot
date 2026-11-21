"""Check real ROS 2 Teleop messages without a keyboard, motors or robot launch."""

import math
from unittest.mock import Mock, patch

import rclpy
from sensor_msgs.msg import JointState

from ai_car import aicar_teleop


def main():
    rclpy.init()
    node = None
    try:
        node = aicar_teleop.AICAR_TELEOP()
        feedback = JointState()
        feedback.header.stamp = node.get_clock().now().to_msg()
        feedback.name = ['tool_joint', 'arm_joint_1', 'arm_joint_2']
        feedback.position = [math.radians(20), math.radians(60), math.radians(30)]
        # Record actual ROS messages, but do not send commands on a robot network.
        with patch.object(node, 'joint_pub', Mock()), patch.object(node, 'tool_pub', Mock()), \
                patch.object(node, 'cmd_pub', Mock()):
            node.cb_joint_states(feedback)
            node._handle_key('u')
            assert list(node.joint_pub.publish.call_args.args[0].data) == [62, 30]
            node.tool_pub.publish.assert_not_called()
            node._handle_key('w')
            assert math.isclose(node.cmd_pub.publish.call_args.args[0].linear.x, 0.01)
            node._handle_key('s')
            assert node.cmd_pub.publish.call_args.args[0].linear.x == 0.0
            print('ROS 2 Teleop message smoke test passed (synthetic feedback)')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
