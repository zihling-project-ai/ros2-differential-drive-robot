import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32


class SpeedMonitor(Node):

    def __init__(self):
        super().__init__('speed_monitor')
        
        self.subscription = self.create_subscription(
            Float32,'/vehicle_speed', self.listener_callback, 10)
        
        self.publisher_ = self.create_publisher(
            Int32, '/penalty_count', 10)
        self.count = 0

    def listener_callback(self, msg):     
        
        if msg.data > 90.0:
            self.count += 1
            penalty_msg = Int32()
            penalty_msg.data = self.count
            self.publisher_.publish(penalty_msg)
            self.get_logger().info(f'[已超速] Speed: {msg.data:.1f} km/h - 累計次數: {self.count} ')
        
        else:
            self.get_logger().info(f'[限速內] Speed: {msg.data:.1f} km/h ')

def main():
    rclpy.init()

    node = SpeedMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
