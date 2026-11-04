import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SpeedSensor(Node):
    def __init__(self):
        super().__init__('speed_sensor')
        self.publishers_ = self.create_publisher(Float32, '/vehicle_speed', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Speed sensor started')

    def timer_callback(self):
        speed = random.uniform(0.0, 140.0)
        msg = Float32()
        msg.data = speed
        self.publishers_.publish(msg)
        self.get_logger().info(
            f'Publishing: {msg.data:.2f} km/h'
        )
        
def main():
    rclpy.init()
    node = SpeedSensor()
    try:
        node.get_logger().info(f'Speed sensor started')
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
