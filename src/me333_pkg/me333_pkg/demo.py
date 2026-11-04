import rclpy
from rclpy.node import Node

class ClassName(Node):
    def __init__(self):
        super().__init__('node_name')

def main():
    rclpy.init()
    node = ClassName()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()