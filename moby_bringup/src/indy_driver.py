#!/usr/bin/python3
#-*- coding: utf-8 -*-
# import sys
import json
import math
import time
from neuromeka import IndyDCP3 
from neuromeka import EtherCAT # for get/set servo rx and get servo tx

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# from indy_interfaces.srv import IndyService
# from indy_interfaces.msg import ServoTx, ServoRx, ServoDataArray

from utils.indy_define import *

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class IndyROSConnector(Node):

    PUBLISH_RATE = 20 # Hz

    def __init__(self):
        super().__init__('indy_driver')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Initialize joint control servers
        self.jtc_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            )

        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_trajectory_callback,
            qos_profile
            )
        self.joint_trajectory_sub  # prevent unused variable warning

        # Initialize topics
        self.timer = self.create_timer(1/self.PUBLISH_RATE, self.timer_callback)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        # Initialize parameters  with default values
        self.declare_parameter('indy_ip', "127.0.0.1")
        self.declare_parameter('indy_type', "indy7")
        self.indy = None
        self.indy_ip = None
        self.indy_type = None
        # self.indy_msg_status = MSG_TELE_STOP

        self.ecat = None
        self.robot_dof = 6
        self.data_per_servo = 5

        # Initialize variable
        # self.vel = 3 # level 1 -> 3
        # self.blend = 0.2 # rad 0 -> 0.4
        self.joint_state_list = []
        self.joint_state_feedback = JointTrajectoryPoint()
        self.execute = False
        self.previous_joint_trajectory_sub = None

        print("Indy connector has been initialised.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    '''
    Connecting to Indy
    '''
    # Connect to Indy
    def connect(self):
        self.indy_ip = self.get_parameter('indy_ip').get_parameter_value().string_value
        self.indy_type = self.get_parameter('indy_type').get_parameter_value().string_value
        print("ROBOT IP: ", self.indy_ip)
        print("ROBOT TYPE: ", self.indy_type)
        self.robot_dof = 7 if (self.indy_type == 'indyrp2' or self.indy_type == 'indyrp2_v2') else 6
        self.indy = IndyDCP3(self.indy_ip)
        self.ecat = EtherCAT(self.indy_ip)

    # Disconnect to Indy
    def disconnect(self):
        print("DISCONNECT TO ROBOT")
        self.indy.stop_teleop()
        time.sleep(1)
        self.indy.stop_motion()
        time.sleep(1)
        del self.indy 

    '''
    Indy subscribe
    '''

    def joint_trajectory_callback(self, msg): # servoing -> teleop
        joint_state_list = []
        if msg.points:
            joint_state_list = [p.positions for p in msg.points]
        else:
            self.indy.stop_motion()
        # print("joint state list: ", joint_state_list) #rad/s rad
        if self.previous_joint_trajectory_sub != joint_state_list[0]:
            # if TELE MODE
            # if self.indy_msg_status == MSG_TELE_JOINT_ABS:
            #     self.indy.movetelej_abs(jpos=rads2degs(joint_state_list[0]))
            self.indy.movej(jtarget=rads2degs(joint_state_list[0]))
            self.previous_joint_trajectory_sub = joint_state_list[0]
    
    '''
    Indy publish
    '''
    # Publish jointstates
    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.indy_type == 'indyrp2' or self.indy_type == 'indyrp2_v2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        control_data = self.indy.get_control_data()
        joint_state_msg.position = degs2rads(control_data['q'])
        joint_state_msg.velocity = degs2rads(control_data['qdot'])
        # joint_state_msg.effort = get_control_torque() TODO
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
        self.get_logger().info('Received goal request!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        print('FollowJointTrajectory callback...')

        result = FollowJointTrajectory.Result()
        feedback_msg = FollowJointTrajectory.Feedback()

        # check if robot is in ILDE mode
        if self.indy.get_control_data()['op_state'] != OP_IDLE:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "ROBOT IS NOT READY"
            return result

        # last_time = self.get_clock().now()
        goal = goal_handle.request.trajectory.points.copy()
        
        # download planned path from ros moveit
        self.joint_state_list = []
        if goal:
            self.joint_state_list = [p.positions for p in goal]
            
        is_cancel = False
        # Do something for OP_IDLE state
        if self.joint_state_list:
            # start teleop
            self.indy.stop_teleop()
            time.sleep(0.1)
            self.indy.start_teleop(method=TELE_JOINT_ABSOLUTE) 
            time.sleep(0.2)

            # wait for telemode actually start
            cur_time = time.time()
            while self.indy.get_control_data()['op_state'] != TELE_OP:
                if (time.time() - cur_time) > 0.5:
                    self.indy.start_teleop(method=TELE_JOINT_ABSOLUTE)  
                    cur_time = time.time()
                time.sleep(0.2)

            # send waypoints
            for j_pos in self.joint_state_list:
                try:
                    self.indy.movetelej_abs(jpos=rads2degs(j_pos))
                except Exception as e:
                    self.get_logger().error('THERE ARE ISSUE WHEN EXECUTE WAYPOINT, PLEASE TRY AGAIN!')
                    is_cancel = True
                    break

                if goal_handle.is_cancel_requested:
                    is_cancel = True
                    break

                feedback_msg.desired.positions = rads2degs(j_pos)
                feedback_msg.actual.positions = self.joint_state_feedback.positions
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05) #20Hz

            time.sleep(0.5) # wait for robot stable

            self.indy.stop_teleop()
            time.sleep(0.3)
            while self.indy.get_control_data()['op_state'] != OP_IDLE:
                time.sleep(0.2)

        if is_cancel:
            goal_handle.canceled()
        else:                
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    with IndyROSConnector() as indy_driver:
        executor = MultiThreadedExecutor()
        rclpy.spin(indy_driver, executor=executor)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if rclpy.ok():
            rclpy.shutdown()
