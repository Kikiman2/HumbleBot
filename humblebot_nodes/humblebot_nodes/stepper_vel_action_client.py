#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humblebot_interfaces.action import Steppers
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle


class StepperVelocitiesClientNode(Node):
    def __init__(self):
        super().__init__("stepper_motor_velocities_client")

        self.declare_parameter("linear_x", 1.0)
        self.declare_parameter("linear_y", 1.0)
        self.declare_parameter("angular_z", 1.0)

        self.linear_x = self.get_parameter("linear_x").value
        self.linear_y = self.get_parameter("linear_y").value
        self.angular_z = self.get_parameter("angular_z").value


        self.stepper_motor_velocities_client_ = ActionClient(
            self,
            Steppers,
            "motor_velocities"
        )

    def send_goal(self, linear_x, linear_y, angular_z):
        # Wait for the server
        self.get_logger().warn("waiting for the server...")
        self.stepper_motor_velocities_client_.wait_for_server()
        self.get_logger().info("Server is up")

        # Create goal
        goal = Steppers.Goal()
        goal.linear_x = linear_x
        goal.linear_y = linear_y
        goal.angular_z = angular_z

        # Send goal
        self.get_logger().info("Sending goal")
        self.stepper_motor_velocities_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"{result.is_finished}")
        if result.is_finished == True:
            self.get_logger().info("Succesfulle reached the goal")
        else:
            self.get_logger().info("Something went wrong")

def main(args=None):
    rclpy.init(args=args)
    node = StepperVelocitiesClientNode()
    node.send_goal(node.linear_x, node.linear_y, node.angular_z)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()