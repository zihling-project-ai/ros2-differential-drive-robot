import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class EchoSubscriber(Node):

    def __init__(self):
        super().__init__('echo_subscriber')
        self.subscription = self.create_subscription(
            Int32,'/my_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received :{msg.data}')


def main():
    rclpy.init()

    node = EchoSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
