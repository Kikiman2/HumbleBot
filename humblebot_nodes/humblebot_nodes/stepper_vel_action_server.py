#!/usr/bin/env python3
import rclpy
import math
import time
import gpiozero
#import asyncio
import threading
from rclpy.node import Node
from humblebot_interfaces.action import Steppers
from rclpy.action.server import ServerGoalHandle

from rclpy.action import ActionServer


class StepperVelocitiesServerNode(Node):
    def __init__(self):
        super().__init__("stepper_motor_velocities_server")
        self.stepper_motor_velocities_server_ = ActionServer(
            self,
            Steppers,
            "motor_velocities",
            execute_callback=self.execute_callback)
        
        self.speed = 0.0002
        self.stepsPerRevolution = 1600
        self.L = 0.160  # Distance from center to wheel axis in mm
        self.wheelDiameter = 73.5 # Wheel diameter in mm
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0
        self.travel_time = 5 # secunds

        # Initialize the GPIO pins
        self.x_step = gpiozero.LED(2)  # x step pin
        self.x_dir = gpiozero.LED(17)    # x direction pin

        self.y_step = gpiozero.LED(3)  # y step pin
        self.y_dir = gpiozero.LED(27)    # y direction pin

        self.z_step = gpiozero.LED(4)  # z step pin
        self.z_dir = gpiozero.LED(22)    # z direction pin

        self.enable = gpiozero.LED(10)  # Enable pin

        self.wheelCircumference = math.pi * self.wheelDiameter

        self.get_logger().info("Motor velocities server has been started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get request from goal
        linear_x = goal_handle.request.linear_x
        linear_y = goal_handle.request.linear_y
        angular_z = goal_handle.request.angular_z

        # Execute the action
        self.get_logger().info("Executing the goal")

        self.calculate_wheel_velocities(linear_x, linear_y, angular_z)

        self.enable_motor()
        
        threads = [
            threading.Thread(target=self.rotate, args=(round(self.velocityToSteps(self.wheel1)), "x")),
            threading.Thread(target=self.rotate, args=(round(self.velocityToSteps(self.wheel2)), "y")),
            threading.Thread(target=self.rotate, args=(round(self.velocityToSteps(self.wheel3)), "z")),
        ]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        # Send final state
        goal_handle.succeed()

        # Send result
        result = Steppers.Result()
        result.is_finished = True
        return result

        
    def calculate_wheel_velocities(self, Vx, Vy, omega):
            """
            Calculate the velocities of each wheel in a three-wheeled omnidirectional robot.

            :param Vx: The orientation angle of the robot (direction of movement).
            :param Vy: The rotational velocity of the robot.
            :param omega: # Distance from the center to each wheel.
            :return: A tuple containing the velocities of the three wheels.
            """
            # Angles of wheels in degrees (60, 180, 270 degrees)

            gamma1 = math.radians(0)
            gamma2 = math.radians(120)
            gamma3 = math.radians(240)

            self.wheel1 = round((Vx * math.cos(gamma1) + Vy * math.sin(gamma1) + self.L * omega),3)
            self.wheel2 = round((Vx * math.cos(gamma2) + Vy * math.sin(gamma2) + self.L * omega),3)
            self.wheel3 = round((Vx * math.cos(gamma3) + Vy * math.sin(gamma3) + self.L * omega),3)

    def velocityToSteps(self, velocity):
        velocity_mm_per_s = velocity * 1000.0 # Convert m/s to mm/s
        distancePerStep = self.wheelCircumference / self.stepsPerRevolution
        return velocity_mm_per_s / distancePerStep

    def enable_motor(self):
        """Enable the motor."""
        self.enable.off()

    def disable_motor(self):
        """Disable the motor."""
        self.enable.on()

    def rotate(self, steps, motor):
        """Rotate the motor a given number of steps."""

        if motor == "x" and steps<0:
            self.x_dir.off()
        elif motor == "x" and steps>0:
            self.x_dir.on()
        elif motor == "y" and steps<0:
            self.y_dir.off()
        elif motor == "y" and steps>0:
            self.y_dir.on()
        elif motor == "z" and steps<0:
            self.z_dir.off()
        elif motor == "z" and steps>0:
            self.z_dir.on()
        else:
            pass

        for _ in range(abs(steps)):
            if motor == "x":
                self.x_step.on()
                time.sleep(self.travel_time/abs(steps))
                self.x_step.off()
                time.sleep(self.travel_time/abs(steps))
            elif motor == "y":
                self.y_step.on()
                time.sleep(self.travel_time/abs(steps))
                self.y_step.off()
                time.sleep(self.travel_time/abs(steps))
            else:
                self.z_step.on()
                time.sleep(self.travel_time/abs(steps))
                self.z_step.off()
                time.sleep(self.travel_time/abs(steps))



def main(args=None):
    rclpy.init(args=args)
    node = StepperVelocitiesServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()