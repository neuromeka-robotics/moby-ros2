#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
import math
import tf2_ros

class SwerveDriveController(Node):
    # PUBLISH_RATE = 50  # Hz

    def __init__(self):
        super().__init__('swerve_drive_controller')

        # cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_gazebo', self.odom_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers velocities and steering angles
        self.traction_pub = self.create_publisher(Float64MultiArray, '/traction_motor_controller/commands', 10)
        self.rotation_pub = self.create_publisher(Float64MultiArray, '/rotation_motor_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Robot-specific
        self.wheel_base = 0.786  # Distance between front and rear wheels
        self.track_width = 0.4108  # Distance between left and right wheels
        self.wheel_radius = 0.1  # Wheel radius (in meters)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Compute steering angles and velocities for each wheel
        wheel_angles = []
        wheel_velocities = []

        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            angle, velocity = self.calculate_wheel_velocity(vx, vy, wz, wheel)
            wheel_angles.append(angle)
            wheel_velocities.append(velocity)

        self.publish_rotation_commands(wheel_angles)
        self.publish_traction_commands(wheel_velocities)

    def calculate_wheel_velocity(self, vx, vy, wz, wheel):
        """
        Calculate steering angle and velocity for the given wheel.
        :param vx: Linear velocity in the x direction (m/s)
        :param vy: Linear velocity in the y direction (m/s)
        :param wz: Angular velocity around the z-axis (rad/s)
        :param wheel: The wheel name (e.g., 'front_left', 'front_right', etc.)
        :return: (wheel_angle, wheel_velocity)
        """

        # Define wheel position based on the wheel name
        if wheel == 'front_left':
            wheel_x, wheel_y = self.wheel_base / 2, self.track_width / 2
        elif wheel == 'front_right':
            wheel_x, wheel_y = self.wheel_base / 2, -self.track_width / 2
        elif wheel == 'rear_left':
            wheel_x, wheel_y = -self.wheel_base / 2, self.track_width / 2
        elif wheel == 'rear_right':
            wheel_x, wheel_y = -self.wheel_base / 2, -self.track_width / 2

        # Calculate the wheel steering angle (in radians) and velocity (m/s)
        wheel_angle = math.atan2(vy + wz * wheel_x, vx - wz * wheel_y)
        wheel_velocity = math.sqrt((vx - wz * wheel_y) ** 2 + (vy + wz * wheel_x) ** 2)

        # Adjust wheel_angle and wheel_velocity to be within -π/2 to π/2 radians (-90 to 90 degrees)
        if wheel_angle > math.pi / 2:
            wheel_angle -= math.pi
            wheel_velocity = -wheel_velocity
        elif wheel_angle < -math.pi / 2:
            wheel_angle += math.pi
            wheel_velocity = -wheel_velocity

        wheel_velocity_rad = wheel_velocity / self.wheel_radius
        return wheel_angle, wheel_velocity_rad


    def publish_rotation_commands(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.rotation_pub.publish(msg)

    def publish_traction_commands(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.traction_pub.publish(msg)
        
    def odom_callback(self, msg):
        self.odom_pub.publish(msg)
        self.broadcast_odometry(msg)

    def broadcast_odometry(self, msg):
        t = TransformStamped()

        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)\

def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
