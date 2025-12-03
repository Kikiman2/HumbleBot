#!/usr/bin/env python3
import rclpy
import math
import time
import warnings
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Suppress gpiozero pin factory warnings
warnings.filterwarnings("ignore", message=".*Falling back from lgpio.*")
import gpiozero



class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")

        # Subscribers and publishers for Twist commands
        self.subscriber_ = self.create_subscription(
            Twist, "/cmd_vel", self.callback_robot_direction, 10)

        self.publisher = self.create_publisher(Twist, '/humblebot/cmd_vel', 10)

        # Stepper motor parameters
        self.speed = 0.00001
        self.stepsPerRevolution = 6400
        self.L = 0.160  # Distance from center to wheel axis in m
        self.wheelDiameter = 73.5  # Wheel diameter in mm
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0

        self.wheel_stepps_x = 0
        self.wheel_stepps_y = 0
        self.wheel_stepps_z = 0

        # Initialize GPIO pins for stepper motors
        self.x_step = gpiozero.LED(3)   # x step pin
        self.x_dir = gpiozero.LED(27)   # x direction pin

        self.y_step = gpiozero.LED(4)   # y step pin
        self.y_dir = gpiozero.LED(22)   # y direction pin

        self.z_step = gpiozero.LED(2)   # z step pin
        self.z_dir = gpiozero.LED(17)   # z direction pin

        self.enable = gpiozero.LED(10)  # Enable pin

        self.wheelCircumference = math.pi * self.wheelDiameter

        self.robot_orientation = 0

        self.get_logger().info("Robot wheel control node has been started")
        

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

        self.enable_motor()
        
        self.wheel_stepps_x = round(self.velocityToSteps(self.wheel1))
        self.get_logger().info(f"Wheel stepps1: {self.wheel_stepps_x}")
        self.wheel_stepps_y = round(self.velocityToSteps(self.wheel2))
        self.get_logger().info(f"Wheel stepps2: {self.wheel_stepps_y}")
        self.wheel_stepps_z = round(self.velocityToSteps(self.wheel3))
        self.get_logger().info(f"Wheel stepps3: {self.wheel_stepps_z}")

        self.rotate()

        self.publish_data()



        
    def calculate_wheel_velocities(self, Vx, Vy, omega):
        """
        Calculate the velocities of each wheel in a three-wheeled omnidirectional robot.

        :param Vx: Linear velocity in x direction (m/s)
        :param Vy: Linear velocity in y direction (m/s)
        :param omega: Angular velocity (rad/s)
        :return: Updates wheel1, wheel2, wheel3 velocities
        """
        # Angles of wheels (60, 180, 300 degrees for omnidirectional setup)
        gamma1 = math.radians(0)
        gamma2 = math.radians(120)
        gamma3 = math.radians(240)

        self.wheel1 = round((Vx * math.sin(gamma1) + Vy * math.cos(gamma1) + self.L * omega), 3)
        self.wheel2 = round((Vx * math.sin(gamma2) + Vy * math.cos(gamma2) + self.L * omega), 3)
        self.wheel3 = round((Vx * math.sin(gamma3) + Vy * math.cos(gamma3) + self.L * omega), 3)

        self.get_logger().info(f"Wheel1: {self.wheel1}")
        self.get_logger().info(f"Wheel2: {self.wheel2}")
        self.get_logger().info(f"Wheel3: {self.wheel3}")

    def velocityToSteps(self, velocity):
        """Convert velocity (m/s) to steps per second for stepper motor."""
        velocity_mm_per_s = velocity * 1000.0  # Convert m/s to mm/s
        distancePerStep = self.wheelCircumference / self.stepsPerRevolution
        return velocity_mm_per_s / distancePerStep

    def rotate(self):
        """Execute stepper motor rotation based on calculated step counts."""
        if self.wheel_stepps_x < 0:
            self.x_dir.off()
        elif self.wheel_stepps_x > 0:
            self.x_dir.on()

        if self.wheel_stepps_y < 0:
            self.y_dir.off()
        elif self.wheel_stepps_y > 0:
            self.y_dir.on()

        if self.wheel_stepps_z < 0:
            self.z_dir.off()
        elif self.wheel_stepps_z > 0:
            self.z_dir.on()
        
        absx = abs(self.wheel_stepps_x)
        absy = abs(self.wheel_stepps_y)
        absz = abs(self.wheel_stepps_z)

        counter_x = 0
        counter_y = 0
        counter_z = 0

        percent = 0.0
        percentx = 0.0
        percenty = 0.0
        percentz = 0.0
        
        max_steps = absx
        if absy > max_steps:
            max_steps = absy
        if absz > max_steps:
            max_steps = absz

        for i in range(max_steps):
            percent = 100 / max_steps * (i)

            if absx > 0:
                percentx = 100 / absx * counter_x
            else:
                percentx = 100.0

            if absy > 0:
                percenty = 100 / absy * counter_y
            else:
                percenty = 100.0
            
            if absz > 0:
                percentz = 100 / absz * counter_z
            else:
                percentz = 100.0

            if percentx <= percent:
                self.x_step.on()
            if percenty <= percent:
                self.y_step.on()
            if percentz <= percent:
                self.z_step.on()
            
            time.sleep(self.speed)

            if percentx <= percent:
                self.x_step.off()
                counter_x += 1
            if percenty <= percent:
                self.y_step.off()
                counter_y += 1
            if percentz <= percent:
                self.z_step.off()
                counter_z += 1

            time.sleep(self.speed)

    def enable_motor(self):
        """Enable stepper motors."""
        self.enable.off()

    def disable_motor(self):
        """Disable stepper motors."""
        self.enable.on()

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
