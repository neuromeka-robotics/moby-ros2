#!/usr/bin/python3
# -*- coding: utf-8 -*-
# """
# Author: Nguyen Pham
# """

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class TimeCount:
    def __init__(self, count=3, timeout=0.5):
        self.count_out = count
        self.timeout = timeout
        self.clear()

    def clear(self):
        self.time_pre = time.time()
        self.count = 0

    def __call__(self):
        if time.time() - self.time_pre > self.timeout:
            self.clear()
        self.count += 1
        return self.count >= self.count_out



class Controller(Node):
    MAX_SPEED = 0.8
    SPEED_CHANGE_STEP = 0.1

    def __init__(self):
        super().__init__('moby_controller')

        # Set up subscriptions
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Set up publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()

        self.cam_pub = self.create_publisher(Twist, 'agri_cam_vel', 10)
        self.cam_vel = Twist()

        self.stop_pub = self.create_publisher(Bool, 'cmd_stop', 10)
        self.zero_pub = self.create_publisher(Bool, 'cmd_set_zero', 10)
        self.zero_time = None
        self.zero_count = TimeCount(count=3, timeout=0.5)

        self.speed_level = self.SPEED_CHANGE_STEP
        self.speed_up = False
        self.speed_down = False

        # 2 type of robot
        self.declare_parameter('moby_type', "moby_rp")

        # - To move: Press **L2 + Left joystick** for moving (non-holonomic)
        # - To move: Press **L2 + Right joystick** for moving (holonomic)
        # - To change speed: Press **R, R2** to change speed. Maximum 0.8 m/s (linear), 0.8 rad/s (angular)
        self.get_logger().info('Press L2 + Right joystick for moving, turning (holonomic)', once=True)
        self.get_logger().info(f'Press Y or B for speed level, maximum {self.MAX_SPEED}m/s (linear), '
                               f'{self.MAX_SPEED}rad/s (angular)', once=True)

        self.moby_type = self.get_parameter('moby_type').get_parameter_value().string_value

    def joy_callback(self, msg):
        # Buttons = [A, B, 0, X, Y, 0, L, R, L2, R2, 0, 0, 0, Ljoy_bt, Rjoy_bt]
        if msg.buttons[0] and msg.buttons[1] and msg.buttons[3] and msg.buttons[4]:  # L - SET ZERO
            if self.zero_count():
                self.zero_pub.publish(Bool(data=True))
        else:
            self.zero_count.clear()

        if msg.buttons[6]: # L - STOP
            self.stop_pub.publish(Bool(data=True))

        elif msg.buttons[8]: # L2 - enable motion

            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.linear.z = 1.0  # use linear.z as decaying priority, last for 1 second
            self.vel.angular.z = 0.0

            # if self.moby_type == 'moby_rp':
            if msg.axes[1] or msg.axes[0]:
                self.vel.linear.x = self.speed_level * msg.axes[1] # Left stick up/down
                self.vel.linear.y = self.speed_level * msg.axes[0] # Left stick left/right
            if msg.axes[2] or msg.axes[3]: # TODO: backward
                #self.vel.linear.x = self.speed_level * msg.axes[3] # Right stick up/down
                self.vel.angular.z = self.speed_level * msg.axes[2] # Right stick left/right
                #if self.vel.linear.x < 0:
                #    self.vel.angular.z = -self.vel.angular.z
                #round(self.vel.linear.x, 2)
                round(self.vel.linear.z, 2)
            if self.moby_type == 'moby_agri':
                self.vel.linear.y = 0.0
                if msg.axes[6] or msg.axes[7]: # TODO: backward
                    self.cam_vel.linear.z = self.speed_level * msg.axes[7]
                    self.cam_vel.angular.z = self.speed_level * msg.axes[6]
                    self.cam_pub.publish(self.cam_vel)
            # elif self.moby_type == 'moby_agri':
            #     self.vel.linear.x = round(self.speed_level * msg.axes[1], 2)
            #     self.vel.angular.z = round(self.speed_level * msg.axes[0], 2)
            #
            # else:
            #     self.get_logger().error(f"Undefined moby type: {self.moby_type}")

            # self.get_logger().info(f"controller command: {self.vel.linear.x:.2f},  {self.vel.linear.y:.2f},  {self.vel.angular.z:.2f}")
            self.vel_pub.publish(self.vel)

        if (msg.buttons[4] == 1 or msg.buttons[7] == 1) and not self.speed_up:
            self.speed_level = round((self.speed_level + self.SPEED_CHANGE_STEP), 1)
            if self.speed_level > self.MAX_SPEED:
                self.speed_level = self.MAX_SPEED
            self.speed_up = True
            self.get_logger().info(f'MAX SPEED: {self.speed_level}', once=False)
        elif not (msg.buttons[4] == 1 or msg.buttons[7] == 1) and self.speed_up:
            self.speed_up = False

        if (msg.buttons[1] == 1 or msg.buttons[9] == 1) and not self.speed_down:
            self.speed_level = round((self.speed_level - self.SPEED_CHANGE_STEP), 1)
            if self.speed_level < self.SPEED_CHANGE_STEP:
                self.speed_level = 0.0
            self.speed_down = True
            self.get_logger().info(f'MAX SPEED: {self.speed_level}', once=False)
        elif not (msg.buttons[1] == 1 or msg.buttons[9] == 1) and self.speed_down:
            self.speed_down = False


def main():
    rclpy.init()

    node = Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
