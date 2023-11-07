#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from example_interfaces.msg import Int16

import math


class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")

        self.subscriber_ = self.create_subscription(
            Int16, "robot_direction", self.callback_robot_direction, 10)

        self.publisher = self.create_publisher(Float32MultiArray, 'float32_multiarray_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_array)
        
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0

        self.Speed = 1.0  # Robot speed
        # self.theta = math.pi / 4  # Orientation angle (45 degrees)
        self.d = 0.1  # Distance from the center to the wheel contact point
        self.robot_orientation = 0

    def publish_array(self):
        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0]  # Modify this list with your desired data
        self.publisher.publish(msg)

    def callback_robot_direction(self, msg):
        robot_direction = msg.data
        velocities = []
        velocities = self.calculate_wheel_velocities(
            self.Speed, robot_direction)
        
        self.get_logger().info("---")
        self.get_logger().info(str(robot_direction))
        self.get_logger().info(str(velocities[0]))
        self.v1 = velocities[0]
        self.get_logger().info(str(velocities[1]))
        self.v2 = velocities[1]
        self.get_logger().info(str(velocities[2]))
        self.v3 = velocities[2]

    def calculate_wheel_velocities(self, robot_speed, robot_angle):

        wheel_angles = (60, 180, 300)

        wheel_velocities = []

        for angle in wheel_angles:

            linear_velocity = robot_speed * \
                math.sin(math.radians(robot_angle-angle))

            # Calculate the wheel velocity
            wheel_velocity = round(linear_velocity, 3)

            wheel_velocities.append(float(wheel_velocity))

        return wheel_velocities


def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
