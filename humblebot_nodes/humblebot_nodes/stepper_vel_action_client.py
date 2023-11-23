#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humblebot_interfaces.action import Steppers
from rclpy.action import ActionClient
#from rclpy.action.client import *


class StepperVelocitiesClientNode(Node):
    def __init__(self):
        super().__init__("stepper_motor_velocities_client")

        self.stepper_motor_velocities_client_ = ActionClient(
            self,
            Steppers,
            "motor_velocities"
        )

    def send_goal(self, linear_x, linear_y, angular_z):
        # Wait for the server
        while not self.stepper_motor_velocities_client_.wait_for_server():
            self.get_logger().warn("Waiting for server...")

        self.get_logger().info("Server is up")

        # Create goal
        val1 = Steppers.Goal()
        val1.linear_x = linear_x
        val2 = Steppers.Goal()
        val2.linear_y = linear_y
        val3 = Steppers.Goal()
        val3.angular_z = angular_z

        # Send goal
        self.get_logger().info("Sending goal")
        self.stepper_motor_velocities_client_.send_goal_async(val1)
        self.stepper_motor_velocities_client_.send_goal_async(val2)
        self.stepper_motor_velocities_client_.send_goal_async(val3)


def main(args=None):
    rclpy.init(args=args)
    node = StepperVelocitiesClientNode()
    node.send_goal(1.0, 0.0, 0.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()