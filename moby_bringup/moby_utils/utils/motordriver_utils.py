###############################################################################
# Predefined variables                                                        #
###############################################################################

# Math
PI = 3.14159265359
PI2 = 2*PI
DEG2RAD = PI / 180
RAD2DEG = 180 / PI

# Operation mode
OP_MODE_NO_MODE = 0x00
OP_MODE_PROFILE_POSITION = 0x01
OP_MODE_VELOCITY = 0x02
OP_MODE_PROFILE_VELOCITY = 0x03
OP_MODE_TORQUE_PROFILE = 0x04
OP_MODE_HOMING = 0x06
OP_MODE_INTERPOLATED_POSITION = 0x07
OP_MODE_CYCLIC_SYNC_POSITION = 0x08
OP_MODE_CYCLIC_SYNC_VELOCITY = 0x09
OP_MODE_CYCLIC_SYNC_TORQUE = 0x0a


### Common TX
TX_STATUSWORD = 0
TX_MODEOPDISP = 1
TX_ACTUALPOS = 2
TX_ACTUALVEL = 3
TX_ACTUALTOR = 4

### Common RX
RX_CTRWRD = 0
RX_MODEOP = 1
RX_TARPOS = 2
RX_TARVEL = 3
RX_TARTOR = 4

## Welcon specific TX and RX
TX_DI = 5
RX_PHYOUT = 5

## Panasonic specific TX and RX
TX_ERRCODE = 5
TX_TOUCHSTAT = 6
TX_TOUCHPOSVAL = 7
TX_DI = 8

RX_MAXTOR = 5
RX_MAXSPD = 6
RX_TOUCHFUNC = 7


def status2string(statusword):
    if (((statusword) & 0x004f) == 0x0000):   # x0xx 0000
        return "NOT_READY"
    elif (((statusword) & 0x004f) == 0x0040): # x1xx 0000
        return "SWITCH_DISABLED"
    elif (((statusword) & 0x006f) == 0x0021): # x01x 0001
        return "READY_SWITCH"
    elif (((statusword) & 0x006f) == 0x0023): # x01x 0011
        return "SWITCHED_ON"
    elif (((statusword) & 0x006f) == 0x0027): # x01x 0111
        return "OPERATION_ENABLED"
    elif (((statusword) & 0x006f) == 0x0007): # x00x 0111
        return "QUICK_STOP"
    elif (((statusword) & 0x004f) == 0x000f): # x0xx 1111
        return "FAULT_REACTION"
    elif (((statusword) & 0x004f) == 0x0008): # x0xx 1000
        return "FAULT"
    else:
        return "UNKNOWN"

    
def status_string(status):
    a = bin(status)
    if len(a) < 8:
        return
    if len(a) == 8:
        a_6 = 0
        a_7 = 0
    else:
        a_6 = int(a[-6])
        a_7 = int(a[-7])
        
    bl = [int(a[-1]), int(a[-2]), int(a[-3]), int(a[-4]), int(a[-5]), a_6, a_7]

    if (bl[0] == 0) and (bl[1] == 0) and (bl[2] == 0) and (bl[3] == 0) and (bl[6] == 1):
        return "Switch On Disabled"
    elif (bl[0] == 1) and (bl[1] == 0) and (bl[2] == 0) and (bl[3] == 0) and (bl[5] == 1) and (bl[6] == 0):
        return "Ready to Switch On"
    elif (bl[0] == 1) and (bl[1] == 1) and (bl[2] == 0) and (bl[3] == 0) and (bl[5] == 1) and (bl[6] == 0):
        return "Switched On"    
    elif (bl[0] == 1) and (bl[1] == 1) and (bl[2] == 1) and (bl[3] == 0) and (bl[5] == 1) and (bl[6] == 0):
        return "Operation Enable"    
    elif (bl[0] == 1) and (bl[1] == 1) and (bl[2] == 1) and (bl[3] == 0) and (bl[5] == 0) and (bl[6] == 0):
        return "Quick Stop Active"    
    elif (bl[0] == 1) and (bl[1] == 1) and (bl[2] == 1) and (bl[3] == 1) and (bl[6] == 0):
        return "Fault Reaction Active"
    elif (bl[0] == 0) and (bl[1] == 0) and (bl[2] == 0) and (bl[3] == 1) and (bl[6] == 0):
        return "Fault"
    
def error_code(mode_op, status_word):
    string_out = []
    if mode_op==OP_MODE_PROFILE_POSITION:
        if (status_word & 0x2000):
            string_out.append("Following error")
        if (status_word & 0x1000):
            string_out.append("Set-point acknowledge")
        if (status_word & 0x0400):
            string_out.append("Target reached")
        
    elif mode_op==OP_MODE_PROFILE_VELOCITY:
        if (status_word & 0x2000):
            string_out.append("Max slippage error")
        if (status_word & 0x1000):
            string_out.append("Speed")
        if (status_word & 0x0400):
            string_out.append("Target reached")
        
    elif mode_op==OP_MODE_CYCLIC_SYNC_POSITION:
        if (status_word & 0x2000):
            string_out.append("Following error")
        if (status_word & 0x1000):
            string_out.append("Drive follows command value")
        
    elif mode_op==OP_MODE_CYCLIC_SYNC_VELOCITY:
        if (status_word & 0x1000):
            string_out.append("Drive follows command value")
        
    elif mode_op==OP_MODE_CYCLIC_SYNC_TORQUE:
        if (status_word & 0x1000):
            string_out.append("Drive follows command value")
    return string_out
        
    

    