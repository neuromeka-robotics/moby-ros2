from moby_grpc_client import GRPCMobyTask as moby_client
import sys, json, grpc, time
from motordriver_utils import *
import time

moby = moby_client('127.0.0.1') # Moby-agri



def print_ecat(idx):
    statusWord = moby.get_moby_txdata(idx).statusWord
    modeOpDisp = moby.get_moby_txdata(idx).modeOpDisp
    actualPosition = moby.get_moby_txdata(idx).actualPosition
    actualVelocity = moby.get_moby_txdata(idx).actualVelocity
    actualTorque = moby.get_moby_txdata(idx).actualTorque
    print("[{}, {}, {}], [{}, {}, {}]".format(status2string(statusWord), error_code(modeOpDisp, statusWord), modeOpDisp, actualPosition, actualVelocity, actualTorque))

while True:
    print('Moby Pos [X, Y, Theta]: {:.3f}m, {:.3f}m, {:.3f} deg'.format(
    moby.get_moby_pose()[0], moby.get_moby_pose()[1], moby.get_moby_pose()[2]*RAD2DEG))
    print('Moby Vel (m/s): {:.3f}, {:.3f}, {:.3f}'.format(moby.get_moby_vel()[0], moby.get_moby_vel()[1], moby.get_moby_vel()[2]))
    # print('Gyro (deg): ', moby.get_gyro_data()[0]*RAD2DEG, moby.get_gyro_data()[1]*RAD2DEG)
    print('Wheel speed (m/s): {:.3f}, {:.3f}'.format(moby.get_drive_speed()[0], moby.get_drive_speed()[1]))
    print("")
    print("")
    print("")
    time.sleep(1)
