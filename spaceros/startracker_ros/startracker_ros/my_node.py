#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("MyNode has been initialized")

    def timer_callback(self):
        self.get_logger().info("Timer callback triggered")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
