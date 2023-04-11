import sys
import json
import grpc
import math

import os

sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "gRPCServerGenPython")))
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "utils")))

from GlobalVariablegRPCServer_pb2_grpc import GRPCGlobalVariableTaskStub
from GlobalVariablegRPCServer_pb2 import *



class GRPCGlobalVariableTask:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC        
        self.channel = grpc.insecure_channel('%s:50060'%step_ip)
        self.stub = GRPCGlobalVariableTaskStub(self.channel)

    
    ## Implementation
    def set_int(self, idx, val):
        return self.stub.SetInt(GInt(idx=idx, val=val))
    
    def get_int(self, idx):
        return self.stub.GetInt(IntVal(val=idx)).val
    
    def set_ints(self, start_idx, end_idx, vals):
        val = vals
        val.insert(0, end_idx)
        val.insert(0, start_idx)
        return self.stub.SetInts(IntVals(val=val))
    
    def get_ints(self, start_idx, end_idx):
        return self.stub.GetInts(IntVals(val=[start_idx, end_idx])).val
    
    
    