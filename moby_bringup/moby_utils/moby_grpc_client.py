import sys
import json
import grpc
import math


import os

sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "gRPCServerGenPython")))
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "utils")))

from MobygRPCServer_pb2_grpc import GRPCMobyTaskStub
from MobygRPCServer_pb2 import *



class GRPCMobyTask:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC
        
        self.channel = grpc.insecure_channel('%s:50051'%step_ip)
        self.stub = GRPCMobyTaskStub(self.channel)

    
    ###### Moby gRPC protocol
    ## Moby EtherCAT data
    def get_moby_txdata(self, slave_num):
        return self.stub.GetMobyTxData(IntVal(val=slave_num))
    
    def get_moby_rxdata(self, slave_num):
        return self.stub.GetMobyRxData(IntVal(val=slave_num))
    
    ## Moby State
    def get_moby_state(self):
        state = self.stub.GetMobyState(Empty())
        return {'is_ready': state.isReady, 'is_moving': state.isMoving, 'is_move_finished':state.isMoveFinished, 'is_emg_pushed':state.isEmgPushed, 'is_error_state':state.isErrorState, 'is_home_pose':state.isHomePose, 'is_resetting':state.isResetting, 'is_imu_avail':state.isIMUAvailable, 'is_program_running':state.isProgramRunning, 'is_program_pause':state.isProgramPause, 'is_rotation_zero':state.isRotationZero}
    
    def get_moby_error_state(self):
        return self.stub.GetMobyErrorState(Empty()).errorState
    
    ## Battery and charging
    
    ## Get Moby's odometry and physical data
    def get_moby_pose(self):
        pose = self.stub.GetMobyPose(Empty())
        return [pose.px, pose.py, pose.pw]
    
    def get_moby_vel(self):
        vel = self.stub.GetMobyVel(Empty())
        return [vel.vx, vel.vy, vel.vw]
    
    def reset_moby_pose(self):
        return self.stub.ResetMobyPose(Empty())
    
    def get_rotation_angle(self):
        return self.stub.GetRotationAngleDeg(Empty())    
    
    def get_drive_speed(self):
        return self.stub.GetDriveSpeed(Empty())
    
    def get_target_vel(self):
        target = self.stub.GetTargetVel(Empty())
        return [target.vx, target.vy, target.vw]
    
    def get_zero(self):
        return self.stub.GetRotationZeroCount(Empty())       
    
    def get_cmode(self):
        return self.stub.GetCMode(Empty()).val    
    
    ## Get sensor data
    def get_gyro_data(self):
        return self.stub.GetGyroData(Empty()).val
    
    def get_imu_data(self):
        data = self.stub.GetGyroFullData(Empty())
        angle = [data.angleX, data.angleY, data.angleZ]
        vel = [data.angleVelX, data.angleVelY, data.angleVelZ]
        acc = [data.linAccX, data.linAccY, data.linAccZ]
        return angle, vel, acc
    
    def reset_gyro(self):
        return self.stub.ResetGyroSensor(Empty()) 
    
    def use_gyro_for_odom(self, use_gyro):
        return self.stub.UseGyroForOdom(BoolVal(val=use_gyro)) 
    
    def get_ir_data(self):
        value = self.stub.GetIRSensorData(Empty())
        return {'ir_front1': value.ir_front1, 'ir_front2': value.ir_front2, 'ir_left1':value.ir_left1, 'ir_left2':value.ir_left2, 'ir_left3':value.ir_left3, 'ir_rear':value.ir_rear, 'ir_right1':value.ir_right1, 'ir_right2':value.ir_right2, 'ir_right3':value.ir_right3}
    
    ## BMS data
    def get_bms(self):
        value = self.stub.GetBMSData(Empty())
        return {'BMS status-1': value.bms_status[0]/10, 'BMS status-2': value.bms_status[1]/10, 'Pack voltage-1':value.pack_volt[0]/100, 'Pack voltage-2':value.pack_volt[1]/100, 'Battery Voltage-1':value.battery_volt[0]/100, 'Battery Voltage-2':value.battery_volt[1]/100, 'Pack current1-1':value.pack_current1[0]/10, 'Pack current1-2':value.pack_current1[1]/10, 'Pack current2-1':value.pack_current2[0]/10, 'Pack current2-2':value.pack_current2[1]/10}
    
    ## Moby motion command
    def set_step_control(self, vx, vy, vw):
        return self.stub.SetStepControl(TargetVel(vx=vx, vy=vy, vw=vw))
    
    def stop_motion(self):
        return self.stub.StopMotion(Empty())
    
    def go_straight(self):
        return self.stub.SetRotationAngleDeg(SwerveDoubles(fl=0, fr=0, bl=0, br=0))
    
    def move_rotation_deg(self, fr, br, bl, fl):
        return self.stub.SetRotationAngleDeg(SwerveDoubles(fr=fr, br=br, bl=bl, fl=fl))
    
    ## Set Moby parameters
    def set_zero_as_current(self):
        return self.stub.SetZeroPosAsCurrentPos(Empty())
    
    def set_rotation_interpolator_vel_acc(self, vel, acc):
        return self.stub.SetRotationVelAcc(DoubleVals(val=[vel, acc]))
    
    def set_drive_interpolator_acc_dec(self, acc, dec):
        return self.stub.SetDriveAccDec(DoubleVals(val=[acc, dec]))
    
    def set_drive_interpolator_onoff(self, onoff):
        return self.stub.SetDriveInterpolatorOnOff(BoolVal(val=onoff))  
    
    def set_rotation_gain_fl(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=0, k=k, kv=kv, kp=kp))  
    
    def set_rotation_gain_fr(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=1, k=k, kv=kv, kp=kp))  
    
    def set_rotation_gain_bl(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=2, k=k, kv=kv, kp=kp))  
    
    def set_rotation_gain_br(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=3, k=k, kv=kv, kp=kp))  
    
    def set_torque_mode(self, isOn):
        return self.stub.SetRotationTorqueMode(BoolVal(val=isOn))  
    
    
    ## Data logger
    def start_logging(self):
        return self.stub.StartRTLogging(Empty())
    
    def end_logging(self):
        return self.stub.EndRTLogging(Empty())
    
    def set_logger(self):
        return self.stub.SetLoggerBuffer(IntVal())
    
    def save_logger(self):
        return self.stub.RTLoggerSave(Empty())
    
    
    
    
    
    
