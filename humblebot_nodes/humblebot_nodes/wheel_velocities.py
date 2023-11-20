#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math



class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")

        self.subscriber_ = self.create_subscription(
            Twist, "/cmd_vel", self.callback_robot_direction, 10)

        self.publisher = self.create_publisher(Twist, '/humblebot/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0

        self.robot_orientation = 0s
        

    def publish_data(self):
        msg2 = Twist()
        msg2.linear.x = float(self.wheel1)
        msg2.linear.y = float(self.wheel2)
        msg2.linear.z = float(self.wheel3)
        self.publisher.publish(msg2)

    def callback_robot_direction(self, msg):
        msg = Twist()
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        self.get_logger().info(str(linear_x))
        self.get_logger().info(str(linear_y))
        velocities = []

        if linear_x == 0.0 and linear_y > 0.0:
            robot_direction = 0
            velocities = self.calculate_wheel_velocities(robot_direction)
            self.wheel1 = velocities[0]
            self.wheel2 = velocities[1]
            self.wheel3 = velocities[2]
        elif linear_x == 0.0 and linear_y < 0.0:
            robot_direction = 180
            velocities = self.calculate_wheel_velocities(robot_direction)
            self.wheel1 = velocities[0]
            self.wheel2 = velocities[1]
            self.wheel3 = velocities[2]
        elif linear_y == 0.0 and linear_x > 0.0:
            robot_direction = 90
            velocities = self.calculate_wheel_velocities(robot_direction)
            self.wheel1 = velocities[0]
            self.wheel2 = velocities[1]
            self.wheel3 = velocities[2]
        elif linear_y == 0.0 and linear_x < 0.0:
            robot_direction = 270
            velocities = self.calculate_wheel_velocities(robot_direction)
            self.wheel1 = velocities[0]
            self.wheel2 = velocities[1]
            self.wheel3 = velocities[2]
        elif linear_y == 0.0 and linear_x == 0.0:
            robot_direction = None
        else:
            robot_direction = math.atan(linear_x/linear_y)
            velocities = self.calculate_wheel_velocities(robot_direction)
            self.wheel1 = velocities[0]
            self.wheel2 = velocities[1]
            self.wheel3 = velocities[2]

    def calculate_wheel_velocities(self, robot_orientation):
        """
        Calculate the velocities of each wheel in a three-wheeled omnidirectional robot.

        :param robot_orientation: The orientation angle of the robot (direction of movement).
        :param angular_velocity: The rotational velocity of the robot.
        :param d: # Distance from the center to each wheel.
        :return: A tuple containing the velocities of the three wheels.
        """
        # Angles of wheels in degrees (0, 120, 240 degrees)
        wheel_angles = [60, 180, 300]

        # Calculate wheel velocities
        wheel_velocities = []

        for angle in wheel_angles:
            # Velocity contribution due to linear speed
            linear_component = round(math.sin(math.radians(angle - robot_orientation)),3)

            wheel_velocities.append(linear_component)

        return wheel_velocities

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
