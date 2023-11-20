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
        #self.timer = self.create_timer(1.0, self.publish_data)
        
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0

        self.robot_orientation = 0
        

    def publish_data(self):
        msg2 = Twist()
        msg2.linear.x = float(self.wheel1)
        msg2.linear.y = float(self.wheel2)
        msg2.linear.z = float(self.wheel3)
        self.publisher.publish(msg2)

    def callback_robot_direction(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        self.calculate_wheel_velocities(linear_x, linear_y, angular_z)

        self.publish_data()



        
    def calculate_wheel_velocities(self, Vx, Vy, omega):
        """
        Calculate the velocities of each wheel in a three-wheeled omnidirectional robot.

        :param robot_orientation: The orientation angle of the robot (direction of movement).
        :param angular_velocity: The rotational velocity of the robot.
        :param d: # Distance from the center to each wheel.
        :return: A tuple containing the velocities of the three wheels.
        """
        # Angles of wheels in degrees (60, 180, 270 degrees)

        L = 0.5  # Distance from center to wheel axis

        gamma1 = math.radians(0)
        gamma2 = math.radians(120)
        gamma3 = math.radians(240)


        self.wheel1 = round((Vx * math.cos(gamma1) + Vy * math.sin(gamma1) + L * omega),3)
        self.wheel2 = round((Vx * math.cos(gamma2) + Vy * math.sin(gamma2) + L * omega),3)
        self.wheel3 = round((Vx * math.cos(gamma3) + Vy * math.sin(gamma3) + L * omega),3)

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
