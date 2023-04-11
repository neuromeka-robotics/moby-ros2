#!/usr/bin/python3
# -*- coding: utf-8 -*-
# import sys
import json
import math
import time

import rclpy

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.action import FollowJointTrajectory

from indy_utils import indydcp_client
from indy_utils import indy_program_maker
from indy_utils import utils_transf


class IndyROSConnector(Node):
    PUBLISH_RATE = 10  # Hz

    def __init__(self):
        super().__init__('indy_driver')

        # Initialize joint control servers
        self.jtc_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_trajectory_callback,
            1)
        # self.joint_trajectory_sub  # prevent unused variable warning

        # Initialize topics
        self.timer = self.create_timer(1 / self.PUBLISH_RATE, self.timer_callback)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Initialize parameters  with default values
        self.declare_parameter('indy_ip', "192.168.214.20")

        # Initialize variable
        self.indy = None
        self.robot_name = None
        self.joint_state_list = []
        self.joint_state_feedback = JointTrajectoryPoint()
        self.execute = False
        self.vel = 5
        self.blend = 5
        self.goal = None
        self.previous_joint_trajectory_sub = None
        self.get_logger().info("Indy connector has been initialised.")

    '''
    Connecting to Indy
    '''

    # Connect to Indy
    def connect(self):
        indy_ip = self.get_parameter('indy_ip').get_parameter_value().string_value
        # print("IndyDCP ROBOT IP: ", indy_ip)
        self.robot_name = "NRMK-Indy7"  # indy7 is default

        # Connect to Robot
        self.indy = indydcp_client.IndyDCPClient(indy_ip, self.robot_name)
        self.indy.connect()

    # Disconnect to Indy
    def disconnect(self):
        self.indy.disconnect()

    '''
    Indy subscribe
    '''

    def joint_trajectory_callback(self, msg):
        joint_state_list = []
        if msg.points:
            joint_state_list = [p.positions for p in msg.points]
        else:
            self.indy.stop_motion()
        if self.previous_joint_trajectory_sub != joint_state_list[0]:
            self.indy.joint_move_to(utils_transf.rads2degs(joint_state_list[0]))
            self.previous_joint_trajectory_sub = joint_state_list[0]

    '''
    Indy publish
    '''

    # Publish jointstates
    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        joint_state_msg.position = utils_transf.degs2rads(self.indy.get_joint_pos())

        # Disable get velocity and torque to speed up publish
        # joint_state_msg.velocity = self.indy.get_joint_vel()
        # joint_state_msg.effort = self.indy.get_control_torque()

        self.joint_state_feedback.positions = joint_state_msg.position

        self.joint_state_pub.publish(joint_state_msg)

    # Timer callback for publish
    def timer_callback(self):
        self.joint_state_publisher()

    '''
    Indy follow joint trajectory 
    '''

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('FollowJointTrajectory callback...')

        feedback_msg = FollowJointTrajectory.Feedback()
        last_time = self.get_clock().now()

        # download planned path from ros moveit
        self.joint_state_list = []
        if self.goal.trajectory.points:
            self.joint_state_list = [p.positions for p in self.goal.trajectory.points]
        else:
            self.indy.stop_motion()

        current_robot_status = self.indy.get_robot_status()
        result = FollowJointTrajectory.Result()

        if current_robot_status['busy']:
            result.error_string = "ROBOT IS BUSY"
            return result

        if current_robot_status['direct_teaching']:
            result.error_string = "ROBOT IS IN TEACHING MODE"
            return result

        if current_robot_status['collision']:
            result.error_string = "ROBOT IS IN COLLISION STATE"
            return result

        if current_robot_status['emergency']:
            result.error_string = "ROBOT IS IN EMERGENCY STATE"
            return result

        if current_robot_status['ready']:
            if self.joint_state_list:

                # prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
                # for j_pos in self.joint_state_list:
                #     prog.add_joint_move_to(utils_transf.rads2degs(j_pos), vel=self.vel, blend=self.blend)
                # json_string = json.dumps(prog.json_program)
                # self.indy.set_and_start_json_program(json_string)

                self.indy.set_joint_vel_level(9)
                for j_pos in self.joint_state_list:
                    self.indy.joint_waypoint_append(utils_transf.rads2degs(j_pos), 0, 5)
                self.indy.joint_waypoint_execute()

                self.joint_state_list = []
                current_robot_status = self.indy.get_robot_status()

                while current_robot_status['busy']:

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info("Trajectory canceled")
                        return FollowJointTrajectory.Result()

                    # print("self.joint_state_feedback.positions: ", self.joint_state_feedback.positions)
                    feedback_msg.desired.positions = self.joint_state_feedback.positions
                    feedback_msg.actual.positions = self.joint_state_feedback.positions
                    goal_handle.publish_feedback(feedback_msg)

                    # print("wait for robot move complete")
                    time.sleep(0.1)
                    current_robot_status = self.indy.get_robot_status()

        goal_handle.succeed()
        result.error_code = 0
        # self.get_logger().info('Returning result: {0}'.format(result.success))
        return result


def main(args=None):
    rclpy.init(args=args)

    indy_driver = IndyROSConnector()
    indy_driver.connect()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(indy_driver, executor=executor)

    indy_driver.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
