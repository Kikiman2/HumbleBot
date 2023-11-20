#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist



class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("direction_publisher")

        self.declare_parameter("linear_x", 0.0)
        self.declare_parameter("linear_y", 0.0)
        self.declare_parameter("angular_z", 0.0)

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5, self.publish_direction)

    def publish_direction(self):
        msg = Twist()
        msg.linear.x = self.get_parameter("linear_x").value
        msg.linear.y = self.get_parameter("linear_y").value
        msg.angular.z = self.get_parameter("angular_z").value
        self.publisher_.publish(msg)
         


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()