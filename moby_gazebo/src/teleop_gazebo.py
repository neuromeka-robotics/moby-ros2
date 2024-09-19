#!/usr/bin/env python3

import sys
import tty
import time
import math
import rclpy
import select
import termios
import threading
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -------KEYBOARD CONFIG----------
KEYCODE_1 = 0x31
KEYCODE_2 = 0x32
KEYCODE_3 = 0x33
KEYCODE_4 = 0x34
KEYCODE_5 = 0x35
KEYCODE_6 = 0x36
KEYCODE_R = 0x72
KEYCODE_Q = 0x71
KEYCODE_EQUAL = 0x3D
KEYCODE_MINUS = 0x2D

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'a': (1.1, 1.1),
    'z': (.9, .9),
    's': (1.1, 1),
    'x': (.9, 1),
    'd': (1, 1.1),
    'c': (1, .9),
}

# Mapping from keycodes to joint names
KEYCODE_TO_JOINTS = {
    KEYCODE_1: "joint0",
    KEYCODE_2: "joint1",
    KEYCODE_3: "joint2",
    KEYCODE_4: "joint3",
    KEYCODE_5: "joint4",
    KEYCODE_6: "joint5",
}

# Limit joint angles
JOINT_LIMITS = (-math.pi, math.pi)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

class KeyboardReader:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def read_one(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Initialize joint positions and velocities
        self.joint_positions = [0.0] * 6  # Initial positions for 6 joints
        self.joint_angular_step = 0.1  # step for joint movement
        self.joint_initialized = False

        # Initialize Twist message parameters
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.reader = KeyboardReader()

    def joint_state_callback(self, msg):
        if not self.joint_initialized:
            for i, name in enumerate(msg.name):
                if name in KEYCODE_TO_JOINTS.values():
                    joint_index = list(KEYCODE_TO_JOINTS.values()).index(name)
                    self.joint_positions[joint_index] = msg.position[i]
            self.joint_initialized = True
            print("Initialized joint positions from /joint_states")

    def send_joint_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start.sec = 1  # Time to reach new position

        msg.points.append(point)
        self.joint_pub.publish(msg)

    def send_twist(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.x * self.speed
        twist_msg.linear.y = self.y * self.speed
        twist_msg.angular.z = self.th * self.turn

        self.twist_pub.publish(twist_msg)

    def key_loop(self):
        try:
            print("Use 1-6 keys to control joints 0-5, 'R' to reverse the direction of joint jogging.")
            print("Use 'u', 'i', 'o', 'j', 'k', 'l', 'm', '<', '>' for velocity control")
            print("Use '+' and '-' to adjust joint step, 'a'/'z' to adjust overall speed.")
            print("'Q' to quit.")
            
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                
                key = self.reader.read_one()

                # Joint control
                if key in [chr(code) for code in KEYCODE_TO_JOINTS]:
                    joint_index = ord(key) - KEYCODE_1  # Map key to joint index (0-5)

                    # Update the joint position and clamp to limits
                    new_position = self.joint_positions[joint_index] + self.joint_angular_step
                    self.joint_positions[joint_index] = clamp(new_position, JOINT_LIMITS[0], JOINT_LIMITS[1])

                    print(f"Moving {KEYCODE_TO_JOINTS[ord(key)]} to position {self.joint_positions[joint_index]}")
                    self.send_joint_trajectory()

                # Velocity control
                elif key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.th = moveBindings[key][3]
                    self.send_twist()

                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print('currently:\tspeed %s\tturn %s ' % (self.speed, self.turn))

                # Reverse joint direction
                elif key == chr(KEYCODE_R):
                    self.joint_angular_step *= -1
                    print("Reversing joint movement direction")

                # Adjust joint speed
                elif key == chr(KEYCODE_EQUAL):
                    self.joint_angular_step = min(self.joint_angular_step + 0.05, 1.0)
                    print(f"Increasing joint step: {self.joint_angular_step}")

                elif key == chr(KEYCODE_MINUS):
                    self.joint_angular_step = max(self.joint_angular_step - 0.05, 0.05)
                    print(f"Decreasing joint step: {self.joint_angular_step}")

                # Quit
                elif key == chr(KEYCODE_Q):
                    print("Exiting...")
                    break

                # Stop if any other key is pressed
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.th = 0.0
                    self.send_twist()

        except Exception as e:
            print(f"Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.key_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
