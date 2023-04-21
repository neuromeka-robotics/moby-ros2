import sys
import json
import grpc
import math
# from bitarray import bitarray
# from bitarray.util import int2ba


import os

sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "gRPCServerGenPython")))
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "utils")))

from MotorControlgRPCServer_pb2_grpc import GRPCMotorControlTaskStub
from MotorControlgRPCServer_pb2 import *



class GRPCMotorControlTask:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC
        
        self.channel = grpc.insecure_channel('%s:50100'%step_ip)
        self.stub = GRPCMotorControlTaskStub(self.channel)

    
    ###### Motor control gRPC protocol
    ## EtherCAT data
    def get_motor_txdata(self, slave_num):
        return self.stub.GetMotorTxPDO(ServoIdx(index=slave_num))
    
    def get_motor_rxdata(self, slave_num):
        return self.stub.GetMotorRxPDO(ServoIdx(index=slave_num))
    
    ## Servo setting
    def set_servo(self):
        return self.stub.SetServo(Empty())
    
    def set_zero_as_current_pos(self, slave_idx):
        return self.stub.SetZeroPosAsCurrent(ServoIdx(index=slave_idx))
    
    ## Get Control data
    def get_control_data(self, slaveIdx):
        data = self.stub.GetControlData(ServoIdx(index=slaveIdx))
        q = data.q
        qdes = data.qdes
        return data
    
    ## Controller and Trajectory setting
    def get_control_gain(self, slave_idx):
        return self.stub.GetControlGain(ServoIdx(index=slave_idx))
    
    def set_control_gain(self, slave_idx, k, kv, kp):
        return self.stub.SetControlGain(ControlGain(index=slave_idx, k=k, kv=kv, kp=kp))
    
    def get_traj_max(self, slave_idx):
        return self.stub.GetTrajMaxValue(ServoIdx(index=slave_idx))
    
    def set_traj_max(self, slave_idx, maxVel, maxAcc):
        return self.stub.SetTrajMaxValue(TrajMax(index=slave_idx, maxVel=maxVel, maxAcc=maxAcc))
    
    def get_position_limit(self, slave_idx):
        return self.stub.GetPositionLimit(ServoIdx(index=slave_idx))
    
    def set_position_limit(self, slave_idx, posMax, posMin):
        return self.stub.SetPositionLimit(PositionLimit(index=slave_idx, posMax=posMax, posMin=posMin))
    
    def get_velocity_limit(self, slave_idx):
        return self.stub.GetVelocityLimit(ServoIdx(index=slave_idx))
    
    def set_velocity_limit(self, slave_idx, velMax, velMin):
        return self.stub.SetVelocityLimit(VelocityLimit(index=slave_idx, velMax=velMax, velMin=velMin))
    
    def get_torque_limit(self, slave_idx):
        return self.stub.GetTorqueLimit(ServoIdx(index=slave_idx))
    
    def set_torque_limit(self, slave_idx, torMax):
        return self.stub.SetTorqueLimit(TorqueLimit(index=slave_idx, torMax=torMax))
    
    ## Motion command
    def move_to(self, slave_idx, targetPos, velLevel, mode, blend):
        return self.stub.MoveTo(MoveCmd(index=slave_idx, targetDegree=targetPos, velLevel=velLevel, moveMode=mode, motionBlend=blend))
    
    def move_by(self, slave_idx, targetPos, velLevel, mode, blend):
        return self.stub.MoveTo(MoveCmd(index=slave_idx, targetDegree=targetPos, velLevel=velLevel, moveMode=mode, motionBlend=blend))
    
    def stop_motion(self, slave_idx):
        return self.stub.StopMotion(ServoIdx(index=slave_idx))
    
    ## Data logger
    def start_logging(self):
        return self.stub.StartRTLogging(Empty())
    
    def end_logging(self):
        return self.stub.EndRTLogging(Empty())
    
    def set_logger(self):
        return self.stub.SetLoggerBuffer(IntVal())
    
    def save_logger(self):
        return self.stub.RTLoggerSave(Empty())
    
    ## For debugging
    def set_cutoff_freq(self, slave_idx, cutoff):
        return self.stub.SetFilterCutoffFreq(ServoIdxVal(index=slave_idx, val=cutoff))
    