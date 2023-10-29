#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

import math


class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")

        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)  # Rewrite needed
        self.get_logger().info("Service has been started!")

        self.S = 1.0  # Robot speed
        self.theta = math.pi / 4  # Orientation angle (45 degrees)
        self.d = 0.1  # Distance from the center to the wheel contact point
        v1, v2, v3 = self.calculate_wheel_velocities(
            self.S, self.theta, self.d)

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))
        return response

    def calculate_wheel_velocities(self, S, theta, d):
        # Linear velocity (V) is the same as the robot's speed.
        V = S

        # Angular velocity (ω) based on the robot's orientation.
        omega = 2 * math.tan(theta)

        # Calculate wheel velocities.
        v1 = V * math.cos(theta) - omega * d
        v2 = V * math.cos(theta - 2 * math.pi / 3) - omega * d
        v3 = V * math.cos(theta + 2 * math.pi / 3) - omega * d

        return v1, v2, v3


def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
