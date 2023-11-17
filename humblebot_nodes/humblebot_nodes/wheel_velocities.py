#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math



class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")

        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.callback_robot_direction, 10)

        self.publisher = self.create_publisher(Twist, '/humble/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0

        self.angular_velocity_ = self.rpm_to_angular_velocity(400)  # Robot angular velocity in rad/s
        self.Speed = 1.0                                            # Robot speed in m/s
        self.robot_orientation = 0                                  # Calculated in radians
        self.d_ = 0.1                                               # Distance from the center to the wheel contact point in meters
        

    def publish_data(self):
        msg = Twist()
        msg.linear.x = self.wheel1
        msg.linear.y = self.wheel2
        msg.linear.z = self.wheel3
        self.publisher.publish(msg)

    def callback_robot_direction(self, msg):
        robot_direction = msg.data
        velocities = []

        # TODO calculate the robot direction from /cmd_vel linear x and y
        velocities = self.calculate_wheel_velocities(self.Speed, robot_direction, self.angular_velocity_, self.d_)
        robot_direction
        self.get_logger().info("---")
        self.get_logger().info(str(robot_direction))
        self.get_logger().info(str(velocities[0]))
        self.wheel1 = velocities[0]
        self.get_logger().info(str(velocities[1]))
        self.v2 = velocities[1]
        self.get_logger().info(str(velocities[2]))
        self.v3 = velocities[2]

    def calculate_wheel_velocities(self, robot_speed, robot_orientation, angular_velocity, d):
        """
        Calculate the velocities of each wheel in a three-wheeled omnidirectional robot.

        :param robot_speed: The linear speed of the robot.
        :param robot_orientation: The orientation angle of the robot (direction of movement).
        :param angular_velocity: The rotational velocity of the robot.
        :param d: # Distance from the center to each wheel.
        :return: A tuple containing the velocities of the three wheels.
        """
        # Angles of wheels in degrees (0, 120, 240 degrees)
        wheel_angles = [0, 120, 240]

        # Convert angles to radians
        robot_orientation_rad = math.radians(robot_orientation)
        wheel_angles_rad = [math.radians(angle) for angle in wheel_angles]

        # Calculate wheel velocities
        wheel_velocities = []

        for angle in wheel_angles_rad:
            # Velocity contribution due to linear speed
            linear_component = robot_speed * math.cos(robot_orientation_rad - angle)

            # Velocity contribution due to rotation
            rotational_component = angular_velocity * d

            # Total wheel velocity
            wheel_velocity = linear_component - rotational_component
            wheel_velocities.append(wheel_velocity)

        return wheel_velocities
    
    def rpm_to_angular_velocity(self, rpm):
        """
        Convert RPM (Revolutions Per Minute) to Angular Velocity (Radians per Second).

        :param rpm: Revolutions Per Minute
        :return: Angular velocity in Radians per Second
        """
        return rpm * (2 * math.pi / 60)

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
