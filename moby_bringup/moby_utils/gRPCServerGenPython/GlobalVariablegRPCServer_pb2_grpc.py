# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import GlobalVariablegRPCServer_pb2 as GlobalVariablegRPCServer__pb2


class GRPCGlobalVariableTaskStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SetInt = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/SetInt',
                request_serializer=GlobalVariablegRPCServer__pb2.GInt.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                )
        self.GetInt = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/GetInt',
                request_serializer=GlobalVariablegRPCServer__pb2.IntVal.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.IntVal.FromString,
                )
        self.SetInts = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/SetInts',
                request_serializer=GlobalVariablegRPCServer__pb2.IntVals.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                )
        self.GetInts = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/GetInts',
                request_serializer=GlobalVariablegRPCServer__pb2.IntVals.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.IntVals.FromString,
                )
        self.SaveGlobalVariables = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/SaveGlobalVariables',
                request_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                )
        self.LoadGlobalVariables = channel.unary_unary(
                '/GRPCGlobalVariable.GRPCGlobalVariableTask/LoadGlobalVariables',
                request_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
                response_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                )


class GRPCGlobalVariableTaskServicer(object):
    """Missing associated documentation comment in .proto file."""

    def SetInt(self, request, context):
        """Global variables
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetInt(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetInts(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetInts(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SaveGlobalVariables(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def LoadGlobalVariables(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_GRPCGlobalVariableTaskServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SetInt': grpc.unary_unary_rpc_method_handler(
                    servicer.SetInt,
                    request_deserializer=GlobalVariablegRPCServer__pb2.GInt.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            ),
            'GetInt': grpc.unary_unary_rpc_method_handler(
                    servicer.GetInt,
                    request_deserializer=GlobalVariablegRPCServer__pb2.IntVal.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.IntVal.SerializeToString,
            ),
            'SetInts': grpc.unary_unary_rpc_method_handler(
                    servicer.SetInts,
                    request_deserializer=GlobalVariablegRPCServer__pb2.IntVals.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            ),
            'GetInts': grpc.unary_unary_rpc_method_handler(
                    servicer.GetInts,
                    request_deserializer=GlobalVariablegRPCServer__pb2.IntVals.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.IntVals.SerializeToString,
            ),
            'SaveGlobalVariables': grpc.unary_unary_rpc_method_handler(
                    servicer.SaveGlobalVariables,
                    request_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            ),
            'LoadGlobalVariables': grpc.unary_unary_rpc_method_handler(
                    servicer.LoadGlobalVariables,
                    request_deserializer=GlobalVariablegRPCServer__pb2.Empty.FromString,
                    response_serializer=GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'GRPCGlobalVariable.GRPCGlobalVariableTask', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class GRPCGlobalVariableTask(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def SetInt(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/SetInt',
            GlobalVariablegRPCServer__pb2.GInt.SerializeToString,
            GlobalVariablegRPCServer__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetInt(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/GetInt',
            GlobalVariablegRPCServer__pb2.IntVal.SerializeToString,
            GlobalVariablegRPCServer__pb2.IntVal.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SetInts(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/SetInts',
            GlobalVariablegRPCServer__pb2.IntVals.SerializeToString,
            GlobalVariablegRPCServer__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetInts(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/GetInts',
            GlobalVariablegRPCServer__pb2.IntVals.SerializeToString,
            GlobalVariablegRPCServer__pb2.IntVals.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SaveGlobalVariables(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/SaveGlobalVariables',
            GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            GlobalVariablegRPCServer__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def LoadGlobalVariables(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GRPCGlobalVariable.GRPCGlobalVariableTask/LoadGlobalVariables',
            GlobalVariablegRPCServer__pb2.Empty.SerializeToString,
            GlobalVariablegRPCServer__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
