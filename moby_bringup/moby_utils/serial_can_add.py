import serial
import time

#serial init
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

#number of Modules init
num_module = 2

#data init
read_data = b''

pack_vol = [b'' for i in range(num_module)]
pack_vol_data = 0

battery_vol = [b'' for i in range(num_module)]
battery_vol_data = 0

pack_1cur = [b'' for i in range(num_module)]
pack_1cur_data = 0

pack_2cur = [b'' for i in range(num_module)]
pack_2cur_data = 0

while True:
    data = ser.read()
    if data == b'\r':
        read_data = b''
    else:
        read_data += data
    if len(read_data) >=26: # Totla Data length 
        if read_data[5]==49:
            if read_data[6]==49: # 0x1100, 0x1101 
                for i in range(num_module):
                    if read_data[8] == 48 + i: # (Battery Module #i)
                        print(read_data) # raw data
                        pack_vol[i] = read_data[12:14]
                        pack_vol[i] += read_data[10:12]
                        pack_vol_data = int(pack_vol[i], base=16)
                        print(f"pack voltage{i} : {pack_vol_data/100}")

                        battery_vol[i] = read_data[16:18]
                        battery_vol[i] += read_data[14:16]
                        battery_vol_data = int(battery_vol[i], base=16)
                        print(f"battery voltage{i} : {battery_vol_data/100}")      
                        
                        pack_1cur[i] = read_data[20:22]
                        pack_1cur[i] += read_data[18:20]
                        pack_1cur_data = int(pack_1cur[i], base=16)
                        print(f"Pack Current #1 {i} : {pack_1cur_data/10}")   

                        pack_2cur[i] = read_data[24:26]
                        pack_2cur[i] += read_data[22:24]
                        pack_2cur_data = int(pack_2cur[i], base=16)
                        print(f"Pack Current #2 {i} : {pack_2cur_data/10}")                             

            # elif read_data[6]==48: # 0x1000, 0x1001 
            #     for i in range(num_module):
            #         if read_data[8] == 48 + i: # (Battery Module #i)
            #             print(read_data) # raw data
            #             pack_vol[i] = read_data[12:14]
            #             pack_vol[i] += read_data[10:12]
            #             pack_vol_data = int(pack_vol[i], base=16)
            #             print(f"pack voltage{i} : {pack_vol_data}")
                        
            #             battery_vol[i] = read_data[16:18]
            #             battery_vol[i] += read_data[14:16]
            #             battery_vol_data = int(battery_vol[i], base=16)
            #             print(f"battery voltage{i} : {battery_vol_data}")

            elif read_data[6]==50: # 0x1200, 0x1201 
                for i in range(num_module):
                    if read_data[8] == 48 + i: # (Battery Module #i)
                        print(read_data) # raw data
                        pack_vol[i] = read_data[12:14]
                        pack_vol[i] += read_data[10:12]
                        pack_vol_data = int(pack_vol[i], base=16)
                        print(f"pack voltage{i} : {pack_vol_data}")
                        
                        battery_vol[i] = read_data[16:18]
                        battery_vol[i] += read_data[14:16]
                        battery_vol_data = int(battery_vol[i], base=16)
                        print(f"battery voltage{i} : {battery_vol_data}")

            # elif read_data[6]==51: # 0x1300, 0x1301 
            #     for i in range(num_module):
            #         if read_data[8] == 48 + i: # (Battery Module #i)
            #             print(read_data) # raw data
            #             pack_vol[i] = read_data[12:14]
            #             pack_vol[i] += read_data[10:12]
            #             pack_vol_data = int(pack_vol[i], base=16)
            #             print(f"pack voltage{i} : {pack_vol_data}")
                        
            #             battery_vol[i] = read_data[16:18]
            #             battery_vol[i] += read_data[14:16]
            #             battery_vol_data = int(battery_vol[i], base=16)
            #             print(f"battery voltage{i} : {battery_vol_data}")

