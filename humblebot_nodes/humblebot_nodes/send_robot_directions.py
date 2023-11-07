#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int16



class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("direction_publisher")
        
        self.msg_data_ = 0

        self.publisher_ = self.create_publisher(Int16, "robot_direction", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)

    def publish_news(self):
        msg = Int16()
        msg.data = self.msg_data_
        self.get_logger().info(str(self.msg_data_))
        self.publisher_.publish(msg)
        self.msg_data_ = self.msg_data_ + 1 
         


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()