#!/usr/bin/env python3
import rclpy
import math
import time
import warnings
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Suppress gpiozero pin factory warnings
warnings.filterwarnings("ignore", message=".*Falling back from lgpio.*")
import gpiozero


class RobotWheelVelocityControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_wheel_control")
        self.motor_group = MutuallyExclusiveCallbackGroup()
        self.odom_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        # Subscriptions / publishers
        self.subscriber_ = self.create_subscription(
            Twist, "/cmd_vel", self.callback_robot_direction, 10,
            callback_group=self.motor_group
        )
        self.debug_pub = self.create_publisher(Twist, "/humblebot/cmd_vel", 10)
        # Odom + TF
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"
        # Robot pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # Latest body twist estimate (for Odometry.twist)
        self.vx_est = 0.0
        self.vy_est = 0.0
        self.w_est = 0.0
        self.last_twist_time = self.get_clock().now()
        # Stepper parameters
        self.speed = 0.00001
        self.stepsPerRevolution = 6400
        self.L = 0.160  # distance from center to wheel axis (m)  <-- must be calibrated
        self.wheelDiameter_mm = 73.5  # mm  <-- must be calibrated
        # Distance traveled per motor step (meters)
        wheel_circumference_mm = math.pi * self.wheelDiameter_mm
        dist_per_step_mm = wheel_circumference_mm / self.stepsPerRevolution
        self.dist_per_step_m = dist_per_step_mm / 1000.0
        # Wheel commanded linear velocities (m/s)
        self.wheel1 = 0.0
        self.wheel2 = 0.0
        self.wheel3 = 0.0
        # Wheel step targets (steps)
        self.wheel_steps_1 = 0
        self.wheel_steps_2 = 0
        self.wheel_steps_3 = 0
        # GPIO
        self.x_step = gpiozero.LED(3)
        self.x_dir = gpiozero.LED(27)
        self.y_step = gpiozero.LED(4)
        self.y_dir = gpiozero.LED(22)
        self.z_step = gpiozero.LED(2)
        self.z_dir = gpiozero.LED(17)
        self.enable = gpiozero.LED(10)
        # Pending executed wheel travel increments (meters)
        self.pending_s1 = 0.0
        self.pending_s2 = 0.0
        self.pending_s3 = 0.0
        # Publish odom + TF at 50 Hz
        self.odom_timer = self.create_timer(
            0.02, self.update_odom_and_tf, callback_group=self.odom_group
        )
        self.get_logger().info("Robot wheel control + step-odometry node started")


    def update_odom_and_tf(self):
        now = self.get_clock().now()

        with self.lock:
            s1 = self.pending_s1
            s2 = self.pending_s2
            s3 = self.pending_s3
            self.pending_s1 = 0.0
            self.pending_s2 = 0.0
            self.pending_s3 = 0.0

        if (s1 != 0.0) or (s2 != 0.0) or (s3 != 0.0):

            dx_body = (s2 - s3) / math.sqrt(3.0)
            dy_body = (2.0 * s1 - s2 - s3) / 3.0
            dtheta = (s1 + s2 + s3) / (3.0 * self.L)

            yaw_mid = self.yaw + 0.5 * dtheta
            c = math.cos(yaw_mid)
            s = math.sin(yaw_mid)
            self.x += dx_body * c - dy_body * s
            self.y += dx_body * s + dy_body * c
            self.yaw += dtheta
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            dt = 0.02
            self.vx_est = dx_body / dt
            self.vy_est = dy_body / dt
            self.w_est = dtheta / dt
            self.last_twist_time = now

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        age = (now - self.last_twist_time).nanoseconds * 1e-9
        if age > 0.2:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0
        else:
            odom.twist.twist.linear.x = float(self.vx_est)
            odom.twist.twist.linear.y = float(self.vy_est)
            odom.twist.twist.angular.z = float(self.w_est)
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)


    def callback_robot_direction(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        w = float(msg.angular.z)

        self.calculate_wheel_velocities(vx, vy, w)
        self.enable_motor()

        self.wheel_steps_1 = int(round(self.velocityToSteps(self.wheel1)))
        self.wheel_steps_2 = int(round(self.velocityToSteps(self.wheel2)))
        self.wheel_steps_3 = int(round(self.velocityToSteps(self.wheel3)))

        self.rotate_and_accumulate_odom()

        dbg = Twist()
        dbg.linear.x = self.wheel1
        dbg.linear.y = self.wheel2
        dbg.linear.z = self.wheel3
        self.debug_pub.publish(dbg)

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

        self.wheel1 = (Vx * math.sin(gamma1) + Vy * math.cos(gamma1) + self.L * omega)
        self.wheel2 = (Vx * math.sin(gamma2) + Vy * math.cos(gamma2) + self.L * omega)
        self.wheel3 = (Vx * math.sin(gamma3) + Vy * math.cos(gamma3) + self.L * omega)

    def velocityToSteps(self, velocity_m_s):
        velocity_mm_s = velocity_m_s * 1000.0
        dist_per_step_mm = (self.dist_per_step_m * 1000.0)
        if dist_per_step_mm <= 0.0:
            return 0.0
        return velocity_mm_s / dist_per_step_mm

    def rotate_and_accumulate_odom(self):
        # Set directions and signs
        sign1 = -1 if self.wheel_steps_1 < 0 else 1
        sign2 = -1 if self.wheel_steps_2 < 0 else 1
        sign3 = -1 if self.wheel_steps_3 < 0 else 1

        if self.wheel_steps_1 < 0:
            self.x_dir.off()
        elif self.wheel_steps_1 > 0:
            self.x_dir.on()

        if self.wheel_steps_2 < 0:
            self.y_dir.off()
        elif self.wheel_steps_2 > 0:
            self.y_dir.on()

        if self.wheel_steps_3 < 0:
            self.z_dir.off()
        elif self.wheel_steps_3 > 0:
            self.z_dir.on()

        abs1 = abs(self.wheel_steps_1)
        abs2 = abs(self.wheel_steps_2)
        abs3 = abs(self.wheel_steps_3)

        max_steps = max(abs1, abs2, abs3)
        if max_steps == 0:
            return

        c1 = c2 = c3 = 0

        t0 = time.perf_counter()
        for i in range(max_steps):
            percent = 100.0 / max_steps * i

            p1 = 100.0 / abs1 * c1 if abs1 > 0 else 100.0
            p2 = 100.0 / abs2 * c2 if abs2 > 0 else 100.0
            p3 = 100.0 / abs3 * c3 if abs3 > 0 else 100.0

            if p1 <= percent:
                self.x_step.on()
            if p2 <= percent:
                self.y_step.on()
            if p3 <= percent:
                self.z_step.on()

            time.sleep(self.speed)

            if p1 <= percent:
                self.x_step.off()
                c1 += 1
            if p2 <= percent:
                self.y_step.off()
                c2 += 1
            if p3 <= percent:
                self.z_step.off()
                c3 += 1

            time.sleep(self.speed)
        _ = time.perf_counter() - t0

        s1 = sign1 * c1 * self.dist_per_step_m
        s2 = sign2 * c2 * self.dist_per_step_m
        s3 = sign3 * c3 * self.dist_per_step_m

        with self.lock:
            self.pending_s1 += s1
            self.pending_s2 += s2
            self.pending_s3 += s3

    def enable_motor(self):
        self.enable.off()

    def disable_motor(self):
        self.enable.on()


def main(args=None):
    rclpy.init(args=args)
    node = RobotWheelVelocityControlServerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.disable_motor()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
