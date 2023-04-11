from ecat_grpc_client import GRPCECatTask as ecat_client
import sys, json, grpc, time
from motordriver_utils import *

ecat = ecat_client('127.0.0.1') # ecat



while True:
    time.sleep(0.1)
    print("Wheel motor")
    
    whl_tx = []
    whl_tx.append(ecat.get_md_txpdo(1))
    whl_tx.append(ecat.get_md_txpdo(3))
    whl_tx.append(ecat.get_md_txpdo(5))
    whl_tx.append(ecat.get_md_txpdo(7))
    for i in range(0, 4):
        print(whl_tx[i], status2string(whl_tx[i][0]), error_code(whl_tx[i][1], whl_tx[i][0]))
    print("")
    print("")


