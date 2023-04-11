#!/usr/bin/python3
# -*- coding: utf-8 -*-

import json
import math
import time

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, LaserScan, Range
from geometry_msgs.msg import Twist, Quaternion, TransformStamped

from moby_bringup.msg import RailSensor, CameraAngle, CameraHeight, OdomRatio

from tf2_ros import TransformBroadcaster

from moby_utils.moby_grpc_client import GRPCMobyTask as moby_client
from moby_utils.ecat_grpc_client import GRPCECatTask as ecat_client
from moby_utils.utils.motordriver_utils import *
from moby_utils.utils.wrap_utils import *
import os
import numpy as np


from threading import Lock
from collections import deque

from collections import namedtuple
import datetime

LogDat = namedtuple("LogDat", ["t", "x", "y", "yaw"])

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class SetStepControlBurstOnStuck:
    def __init__(self, moby_client, timeout_burst=1.0, count_burst=20):
        self.moby_client = moby_client
        self.set_step_control_raw = moby_client.set_step_control
        self.burst_tic = time.time()
        self.count = 0
        self.timeout_burst = timeout_burst
        self.count_burst = count_burst

    def __call__(self, vx, vy, vw):
        vel = self.moby_client.get_moby_vel()
        if (np.sum(np.abs([vx, vy, vw])) > 0.01
                and np.sum(np.abs(vel)) < 0.001):
            self.count += 1
        else:
            self.burst_tic = time.time()
            self.count = 0
        if self.burst_tic - time.time() > self.timeout_burst \
                and self.count > self.count_burst:
            vx = vx * 3
            vy = vy * 3
            vw = vw * 3
        self.set_step_control_raw(vx, vy, vw)

##
# @class TimedPriority
# @brief time-decaying priority. decays down to zero as time passes, in second units.
class TimedPriority:
    def __init__(self, value=0):
        self.set(value)

    def set(self, value):
        self.time0 = time.time()
        self.value = value

    def __call__(self):
        return max(0, self.value - (time.time() - self.time0))


class MobyROSConnector(Node):
    PUBLISH_RATE = 60  # Hz

    def __init__(self):
        super().__init__('moby_driver')

        # Initialize parameters  with default values
        self.declare_parameter('step_ip', "192.168.214.20")
        self.declare_parameter('use_gyro', True)
        self.declare_parameter('moby_type', "moby_rp")
        self.declare_parameter('body_length', 0.9)
        self.declare_parameter('body_width', 0.6)
        self.declare_parameter('lidar_margin', 0.2)
        self.declare_parameter('ir_margin', 0.2)
        self.declare_parameter('flag_save_log', False)
        self.declare_parameter('duration_log', 30.0)
        self.moby_type = self.get_parameter('moby_type').get_parameter_value().string_value
        self.body_length = self.get_parameter('body_length').get_parameter_value().double_value
        self.body_width = self.get_parameter('body_width').get_parameter_value().double_value
        self.lidar_margin = self.get_parameter('lidar_margin').get_parameter_value().double_value
        self.ir_margin = self.get_parameter('ir_margin').get_parameter_value().double_value
        self.flag_save_log = self.get_parameter('flag_save_log').get_parameter_value().bool_value
        self.reset_log()
        self.vx_max_ir, self.vx_min_ir = np.inf, -np.inf
        self.vy_max_ir, self.vy_min_ir = np.inf, -np.inf
        self.vx_max_lidar, self.vx_min_lidar = np.inf, -np.inf
        self.vy_max_lidar, self.vy_min_lidar = np.inf, -np.inf
        self.scan_ranges_stack = deque(maxlen=2)
        self.priority_saved = TimedPriority()
        self.twist_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        self.stop_subscriber = self.create_subscription(
            Bool,
            'cmd_stop',
            self.stop_callback,
            10
        )

        self.zero_subscriber = self.create_subscription(
            Bool,
            'cmd_set_zero',
            self.zero_callback,
            10
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        if self.moby_type == 'moby_agri':
            self.cam_subscriber = self.create_subscription(
                Twist,
                'agri_cam_vel',
                self.cam_callback,
                10
            )
            # self.twist_subscriber # prevent unused variable warning

            self.camera_module_angle_subscriber = self.create_subscription(
                CameraAngle,
                'camera_angle',
                self.camera_angle_callback,
                10
            )

            self.camera_module_height_subscriber = self.create_subscription(
                CameraHeight,
                'camera_height',
                self.camera_elevator_callback,
                10
            )

        self.odom_ratio_subscriber = self.create_subscription(
            OdomRatio,
            'odom_ratio',
            self.odom_ratio_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom_encoders',
            10
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            'imu',
            10
        )

        self.rail_sensor_publisher = self.create_publisher(
            RailSensor,
            'rail_sensor',
            10
        )

        self.imu_msg = Imu()
        self.odom_msg = Odometry()
        self.rail_msg = RailSensor()

        # Initialize topics
        self.timer = self.create_timer(1 / self.PUBLISH_RATE, self.timer_callback)

        # Initialize variable
        self.moby = None
        self.control_timeout = 0
        self.stop_send_cmd_vel = True

        self.ecat = None
        self.cam_timeout = 0
        self.stop_send_cam_vel = True
        self.cam_lock = Lock()

        self.odom_ratio = 1
        self.use_gyro = False
        self.ir_pub_dict = None

    def reset_log(self):
        if self.flag_save_log:
            self.save_log()
        self.flag_save_log = self.get_parameter('flag_save_log').get_parameter_value().bool_value
        self.cmd_log = []
        self.vel_log = []
        self.pos_log = []

    def save_log(self):
        home_dir = os.environ["HOME"]
        log_dir = os.path.join(home_dir, "LOG", datetime.datetime.now().strftime('%Y%m%d%H%M%S'))
        os.makedirs(log_dir)
        if hasattr(self, "cmd_log") and len(self.cmd_log)>0:
            np.savetxt(os.path.join(log_dir, "cmd_log.csv"), self.cmd_log, delimiter=",")
        if hasattr(self, "vel_log") and len(self.vel_log)>0:
            np.savetxt(os.path.join(log_dir, "vel_log.csv"), self.vel_log, delimiter=",")
        if hasattr(self, "pos_log") and len(self.pos_log)>0:
            np.savetxt(os.path.join(log_dir, "pos_log.csv"), self.pos_log, delimiter=",")

    '''
    Connecting to Moby
    '''

    # Connect to Moby
    def connect(self):
        step_ip = self.get_parameter('step_ip').get_parameter_value().string_value
        self.use_gyro = self.get_parameter('use_gyro').get_parameter_value().bool_value
        self.moby_type = self.get_parameter('moby_type').get_parameter_value().string_value
        self.get_logger().info(f"STEP IP: {step_ip}")
        self.get_logger().info(f"Use gyro: {self.use_gyro}")
        self.get_logger().info(f"Wait Moby on : {step_ip}")
        self.moby = None
        while self.moby is None:
            time.sleep(1)
            try:
                # Connect moby client
                self.moby = moby_client(step_ip)
                self.moby.use_gyro_for_odom(self.use_gyro and self.moby.get_moby_state()['is_imu_avail'])
                self.moby.reset_gyro()
                ir_data = self.moby.get_ir_data()
                if self.ir_pub_dict is None:
                    self.ir_pub_dict = {
                        ir_key: self.create_publisher(Range, f"{ir_key}_range", 10)
                        for ir_key in ir_data.keys()}
                # self.moby.set_step_control = SetStepControlBurstOnStuck(self.moby)
            except Exception as e:
                self.get_logger().error(f"CANNOT CONNECT TO STEP ON {step_ip}. TRY RECONNECT EVERY SECONDS")
                self.get_logger().error(str(e))
                self.moby = None


        self.ecat = None
        while self.ecat is None:
            time.sleep(1)
            try:
                # Connect ecat client
                self.ecat = ecat_client(step_ip)
                if self.moby_type == "moby_agri":
                    slave_num = len(self.ecat.is_system_ready())
                    if slave_num > 5:
                        self.ecat.set_servo(4, False)  # servo off elevator
                        self.ecat.set_servo(3, True)  # servo on rotator
                        self.ecat.set_md_rx_pdo(2, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)

                        self.ecat.set_max_torque(2, 4000)
                        self.ecat.set_max_motor_speed(2, 5000)

                        self.ecat.set_max_torque(3, 65000)
                        self.ecat.set_max_motor_speed(3, 10000000)

                        assert self.ecat.is_system_ready()[4] == 0
                    else:
                        self.get_logger().warn(f"Slave number {slave_num} is not expected for MOBY-AGRI. "
                                               f"Maybe Camera module is not available")
            except Exception as e:
                self.get_logger().error(f"CANNOT CONNECT TO ECAT CLIENT ON {step_ip}")
                self.get_logger().error(str(e))
                self.ecat = None
        self.get_logger().info(f"Moby Connected to : {step_ip}")

    @try_wrap()
    def publish_ir(self):
        ir_data = self.moby.get_ir_data()
        vx_max_ir, vx_min_ir, vy_max_ir, vy_min_ir = np.inf, -np.inf, np.inf, -np.inf
        for ir_key, ir_val in ir_data.items():
            self.ir_pub_dict[ir_key].publish(
                Range(header=Header(frame_id=f"{ir_key}_link",
                                    stamp=self.get_clock().now().to_msg()),
                      radiation_type=Range.INFRARED,
                      field_of_view=0.436,  # deg2rad(25)
                      min_range=0.0,
                      max_range=0.3,
                      range=ir_val / 1000
                      )
            )
            if ir_val / 1000 < self.ir_margin:
                if 'front' in ir_key:
                    vx_max_ir = 0
                if 'rear' in ir_key:
                    vx_min_ir = 0
                if 'left' in ir_key:
                    vy_max_ir = 0
                if 'right' in ir_key:
                    vy_min_ir = 0
        self.vx_max_ir, self.vx_min_ir = vx_max_ir, vx_min_ir
        self.vy_max_ir, self.vy_min_ir = vy_max_ir, vy_min_ir

    @try_wrap()
    def lidar_callback(self, scan: LaserScan):
        self.scan = scan
        scan_angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment)[:-1]
        self.scan_ranges_stack.append(np.array(self.scan.ranges))
        self.scan_ranges = np.min(self.scan_ranges_stack, axis=0)
        self.scan_xy = np.multiply([np.cos(scan_angles), np.sin(scan_angles)],
                                   self.scan_ranges).transpose()
        idx_fr = np.abs(self.scan_xy[:, 1]) < self.body_width / 2
        idx_lr = np.abs(self.scan_xy[:, 0]) < self.body_length / 2
        idx_front = np.logical_and(idx_fr, self.scan_xy[:, 0] > 0)
        idx_rear = np.logical_and(idx_fr, self.scan_xy[:, 0] < 0)
        idx_left = np.logical_and(idx_lr, self.scan_xy[:,1] > 0)
        idx_right = np.logical_and(idx_lr, self.scan_xy[:,1] < 0)

        if np.sum(np.abs(self.scan_xy[idx_front, 0]) < self.body_length / 2 + self.lidar_margin) > 1:
            self.vx_max_lidar = 0
        else:
            self.vx_max_lidar = np.inf

        if np.sum(np.abs(self.scan_xy[idx_rear, 0]) < self.body_length / 2 + self.lidar_margin) > 1:
            self.vx_min_lidar = 0
        else:
            self.vx_min_lidar = -np.inf

        if np.sum(np.abs(self.scan_xy[idx_left, 1]) < self.body_width / 2 + self.lidar_margin) > 1:
            self.vy_max_lidar = 0
        else:
            self.vy_max_lidar = np.inf

        if np.sum(np.abs(self.scan_xy[idx_right, 1]) < self.body_width / 2 + self.lidar_margin) > 1:
            self.vy_min_lidar = 0
        else:
            self.vy_min_lidar = -np.inf

    @try_wrap()
    def stop_callback(self, req: Bool):
        if self.moby is not None and req.data:
            self.get_logger().info(f"STOP MOTION (TORQUE ZERO)")
            self.moby.stop_motion()
            self.reset_log()

    @try_wrap()
    def zero_callback(self, req: Bool):
        if self.moby is not None and req.data:
            self.get_logger().info(f"SET ZERO AS CURRENT")
            self.moby.set_zero_as_current()

    '''
    Moby subscribe
    use linear.z as priority, last z is recorded and attenuated by time in second units to zero
    '''
    @try_wrap()
    def twist_callback(self, twist: Twist):
        if self.moby is not None:
            vx = np.clip(twist.linear.x,
                         max(self.vx_min_ir, self.vx_min_lidar),
                         min(self.vx_max_ir, self.vx_max_lidar))
            vy = np.clip(twist.linear.y,
                         max(self.vy_min_ir, self.vy_min_lidar),
                         min(self.vy_max_ir, self.vy_max_lidar))
            priority = twist.linear.z
            vw = twist.angular.z
            priority_saved = self.priority_saved()
            self.get_logger().debug(f"Set Moby Velocity {[vx, vy, vw]} ({priority:.2f}/{priority_saved:.2f})")
            if priority >= priority_saved:
                self.get_logger().info(f"Set Moby Velocity {[vx, vy, vw]} ({priority:.2f})")
                self.priority_saved.set(priority)
                self.moby.set_step_control(vx, vy, vw)
        self.control_timeout = time.time()
        self.stop_send_cmd_vel = False
        if self.moby is not None and self.flag_save_log:
            stamp = self.get_clock().now().to_msg()
            time_sec = stamp.sec + stamp.nanosec / 1e9
            self.cmd_log.append(LogDat(time_sec, vx, vy, vw))

    def elev_by(self, delta_cm):
        slave_idx = 3
        if self.ecat.is_system_ready()[slave_idx+1] == 0:  # servo on if not
            self.ecat.set_servo(slave_idx+1, True)
            self.ecat.set_md_rx_pdo(slave_idx, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)
        elev_cur = self.ecat.get_md_tx_pdo(3)[2] / 4000000
        target_pos_cm = elev_cur + delta_cm
        # if target_pos_cm < 0:
        #     target_pos_cm = 0
        # if target_pos_cm > 50:
        #     target_pos_cm = 50

        target_pos_cnt = int(target_pos_cm * 4000000)
        self.ecat.set_md_rx_pdo(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
        time.sleep(0.01)
        self.ecat.set_md_rx_pdo(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)

    def rotate_by(self, delta_deg):
        slave_idx = 2
        rot_cur = self.ecat.get_md_tx_pdo(slave_idx)[2] / 1000
        target_pos_deg = rot_cur + delta_deg
        if target_pos_deg > 90:
            target_pos_deg = 90
        if target_pos_deg < -90:
            target_pos_deg = -90

        target_pos_cnt = int(target_pos_deg*1000)

        self.ecat.set_md_rx_pdo(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
        time.sleep(0.01)
        self.ecat.set_md_rx_pdo(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)

    @try_wrap()
    def cam_callback(self, twist: Twist):
        if self.moby_type == 'moby_agri':
            with self.cam_lock:
                if self.ecat is not None:
                    self.get_logger().info(f"Move Camera Elev: {twist.linear.z:.2f}, Rot: {twist.angular.z:.2f}")
                    if abs(twist.linear.z) > 0.05:
                        self.elev_by(twist.linear.z * 20)  # move by 20 cm step

                    if abs(twist.angular.z) > 0.05:
                        self.rotate_by(-twist.angular.z * 45)  # move by 30 deg

                self.cam_timeout = time.time()
                self.stop_send_cam_vel = False

    @try_wrap()
    def odom_ratio_callback(self, msg: OdomRatio):
        self.odom_ratio = msg.odom_ratio
        self.get_logger().info('Odom Ratio: "%f"' % self.odom_ratio)

    @try_wrap()
    def camera_angle_callback(self, msg: CameraAngle):
        self.get_logger().info('Publishing: "%d"' % msg.camera_angle)
        sync_mode = False
        target_pos_deg = msg.camera_angle
        slave_idx = 2
        target_pos_cnt = int(target_pos_deg * 1000)
        if sync_mode:
            self.ecat.set_md_rx_pdo(slave_idx, 0x1f, 1, target_pos_cnt, 0, 0)
        else:
            self.ecat.set_md_rx_pdo(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_md_rx_pdo(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)

    @try_wrap()
    def camera_elevator_callback(self, msg: CameraHeight):
        self.get_logger().info('Publishing: "%d"' % msg.camera_height)
        sync_mode = False
        target_pos_cm = msg.camera_height
        if target_pos_cm > 50:
            self.get_logger().info('TARGET POSITION IS TOO HIGH')
            return
        max_torque = 4000  # 100%
        max_motor_speed = 5000  # Tuning
        slave_idx = 3
        target_pos_cnt = int(target_pos_cm * 4000000)
        print(target_pos_cnt)
        if sync_mode:
            self.ecat.set_md_rx_pdo(slave_idx, 0x1f, 1, target_pos_cnt, 0, 0)
        else:
            self.ecat.set_md_rx_pdo(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_md_rx_pdo(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)

    '''
    Moby publish
    '''
    # Publish jointstates
    @try_wrap()
    def odom_publish_callback(self):
        if self.moby is not None:
            moby_pose = self.moby.get_moby_pose()  # x, y, theta
            moby_vel = self.moby.get_moby_vel()  # x, y, theta
        else:
            moby_pose = [0.0, 0.0, 0.0]
            moby_vel = [0.0, 0.0, 0.0]

        # print("moby pose and vel: ", moby_pose, " ", moby_vel)

        # odometry
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = moby_pose[0]
        self.odom_msg.pose.pose.position.y = moby_pose[1]
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, moby_pose[2])
        self.odom_msg.twist.twist.linear.x = moby_vel[0] * self.odom_ratio
        self.odom_msg.twist.twist.linear.y = moby_vel[1] * self.odom_ratio
        self.odom_msg.twist.twist.angular.z = moby_vel[2]
        if self.flag_save_log:
            time_sec = self.odom_msg.header.stamp.sec + self.odom_msg.header.stamp.nanosec / 1e9
            self.vel_log.append(LogDat(time_sec, *moby_vel))
            self.pos_log.append(LogDat(time_sec, *moby_pose))

        self.odom_publisher.publish(self.odom_msg)

    @try_wrap()
    def imu_publish_callback(self):
        if self.moby is not None:
            if self.use_gyro:
                if self.moby.get_moby_state()['is_imu_avail']:
                    self.moby.use_gyro_for_odom(True)
                    moby_gyro = self.moby.get_gyro_data()
                else:
                    self.moby.use_gyro_for_odom(False)
                    moby_gyro = [self.moby.get_moby_pose()[2], 0]
                    self.get_logger().error("IMU is not available")
            else:
                self.moby.use_gyro_for_odom(False)  # call this everytime, considering STEP3 is restarted
                moby_gyro = [self.moby.get_moby_pose()[2], 0]
        else:
            moby_gyro = [0.0, 0.0]

        # print("moby_gyro: ", moby_gyro)

        # imu
        self.imu_msg.header.frame_id = 'imu_link'
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.orientation = quaternion_from_euler(0, 0, moby_gyro[0])

        self.imu_publisher.publish(self.imu_msg)

    @try_wrap()
    def rail_sensor_publish_callback(self):
        if self.ecat is not None:
            di = self.ecat.get_ioboard_di()
            left_sensor = di[2]
            right_sensor = di[3]
        else:
            left_sensor = 0
            right_sensor = 0

        if left_sensor or right_sensor:  # TODO: Left sensor broken => use or
            self.rail_msg.rail_sensor = 1
        else:
            self.rail_msg.rail_sensor = 0
        self.rail_sensor_publisher.publish(self.rail_msg)
        # self.get_logger().info('Publishing: "%d %d"' %rail_sensor[0] %rail_sensor[1])
        # self.get_logger().info('Publishing: "%d %d"' %(left_sensor, right_sensor))

        # Timer callback for publish

    @try_wrap()
    def timer_callback(self):
        # pass
        self.odom_publish_callback()
        self.imu_publish_callback()
        self.rail_sensor_publish_callback()
        if self.moby is not None:
            if time.time() - self.control_timeout >= 0.15 and not self.stop_send_cmd_vel:
                self.get_logger().info(f"Set Moby Velocity {[0, 0, 0]}")
                self.moby.set_step_control(0.0, 0.0, 0.0)
                self.moby.set_step_control(0.0, 0.0, 0.0)
                self.stop_send_cmd_vel = True
        if self.ecat is not None:
            if self.moby_type == "moby_agri":
                if time.time() - self.cam_timeout >= 0.15 and not self.stop_send_cam_vel:
                    self.elev_by(0)
                    self.rotate_by(0)
                    self.ecat.set_servo(4, False)  # servo off elevator
                    self.ecat.set_servo(3, True)
                    self.ecat.set_md_rx_pdo(2, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)
                    self.stop_send_cam_vel = True
        self.publish_ir()


def main(args=None):
    rclpy.init(args=args)

    moby_driver = MobyROSConnector()
    moby_driver.connect()

    rclpy.spin(moby_driver)

    moby_driver.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
