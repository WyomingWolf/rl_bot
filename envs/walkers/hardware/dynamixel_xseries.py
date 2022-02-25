# *******************************************************************************
#  dynamixel_xseries.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 12/08/2021
# *******************************************************************************

import os
import time
import numpy as np
import multiprocessing as mp

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from envs.walkers.hardware.dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                      = '/dev/ttyUSB0'

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION                = 2.0
BAUDRATE                        = 4000000

ADDR_TORQUE_ENABLE              = 64
ADDR_LED_RED                    = 65
ADDR_HARDWARE_ERROR             = 70
LEN_HARDWARE_ERROR              = 1         # Data Byte Length
ADDR_GOAL_POSITION              = 116
LEN_GOAL_POSITION               = 4         # Data Byte Length
ADDR_PRESENT_LOAD		        = 126
LEN_PRESENT_LOAD		        = 2	    # Data Byte Length
ADDR_PRESENT_VELOCITY		    = 128
LEN_PRESENT_VELOCITY		    = 4	    # Data Byte Length         
ADDR_PRESENT_POSITION           = 132
LEN_PRESENT_POSITION            = 4         # Data Byte Length
ADDR_PRESENT_TEMP               = 146
LEN_PRESENT_TEMP                = 1         # Data Byte Length
ADDR_INDIRECTADDRESS_FOR_READ   = 168
LEN_INDIRECTDATA_FOR_READ       = 12        # Sum of Data of Length. i.e) Hardware Error (1byte) + Present Load (2 byte) + Present Velocity (4 byte) + Present Position (4 bytes) + Present Temperature (1 byte) 
ADDR_INDIRECTDATA_FOR_READ      = 224

TORQUE_DISABLE                  = 0         # Value for disabling the torque
TORQUE_ENABLE                   = 1         # Value for enabling the torque
LED_OFF                         = 0         # Dynamixel LED will light between this value
LED_ON                          = 1         # and this value

LOAD_PRECISION                  = 1000
VEL_PRECISION                   = 1023
POS_PRECISION                   = 4095/2


class XSeries:
    def __init__(self, servos):
        #self.servos = servos
        self.num_servos = np.shape(servos)[0]
        self.ids = servos[:,0]
        self.limits = servos[:,1:3]
        self.reset_steps = servos[:,3:]
        self.num_reset_steps = np.shape(self.reset_steps)[1]

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTDATA_FOR_READ)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()
        
        # reboot all servos
        for i in range(self.num_servos):
            DXL_ID = self.ids[i]
            result =  self.reboot(DXL_ID) 
            attemps = 0
            while result!=0 and attemps<5:
                attemps += 1
                result = self.reboot(DXL_ID)
       

    def setTorque(self, DXL_ID, status):
        # Disable Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, status)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
            #print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def setLED(self, DXL_ID, status):
        # Turn off LED
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_LED_RED, status)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
            #print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def setupServo(self, DXL_ID):         
        # INDIRECTDATA parameter storages replace hardware error status, present load, present velocity, present position and present temperature
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ, ADDR_HARDWARE_ERROR)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRESENT_LOAD)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRESENT_LOAD + 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 8, ADDR_PRESENT_VELOCITY + 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 10, ADDR_PRESENT_VELOCITY + 2)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 12, ADDR_PRESENT_VELOCITY + 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 14, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 16, ADDR_PRESENT_POSITION + 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 18, ADDR_PRESENT_POSITION + 2)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 20, ADDR_PRESENT_POSITION + 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 22, ADDR_PRESENT_TEMP)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Add parameter storage for multiple values
        dxl_addparam_result = self.groupSyncRead.addParam(DXL_ID)
        if dxl_addparam_result == True:   
            print("Dynamixel#%d: Online" % DXL_ID)


    def readState(self):
        fail = False
        dxl_error = np.zeros((self.num_servos,1), dtype=np.int8)
        dxl_load = np.zeros((self.num_servos,1), dtype=np.int16)	
        dxl_state = np.zeros((self.num_servos,2), dtype=np.int32)
        dxl_temp = np.zeros((self.num_servos,1), dtype=np.int8)
        # Syncread present position from indirectdata
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print("Group sync read failed")
            fail = True
        else:
            for i in range(self.num_servos):
                DXL_ID = self.ids[i]
                
                '''           
                # Check if groupsyncread data of Dynamixel error status is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(DXL_ID, ADDR_INDIRECTDATA_FOR_READ, LEN_HARDWARE_ERROR)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    quit()

                # Check if groupsyncread data of Dynamixel present load value is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR, LEN_PRESENT_LOAD)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    quit()

                # Check if groupsyncread data of Dynamixel present velocity value is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD, LEN_PRESENT_VELOCITY)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    quit()

                # Check if groupsyncread data of Dynamixel present Position value is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD + LEN_PRESENT_VELOCITY, LEN_PRESENT_POSITION)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    quit()

                # Check if groupsyncread data of Dynamixel present temperature is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION, LEN_PRESENT_TEMP)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    quit()
                '''

                try:
                    # Get Dynamixel present error status
                    dxl_error[i] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ, LEN_HARDWARE_ERROR)

                    # Get Dynamixel present load value
                    dxl_load[i] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR, LEN_PRESENT_LOAD)

                    # Get Dynamixel present position value
                    dxl_state[i,0] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD, LEN_PRESENT_VELOCITY)

                    # Get Dynamixel present position value
                    dxl_state[i,1] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD + LEN_PRESENT_VELOCITY, LEN_PRESENT_POSITION)

                    # Get Dynamixel present error status
                    dxl_temp[i] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION, LEN_PRESENT_TEMP)
                except Exception as e:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                    fail = True
            dxl_load = dxl_load/LOAD_PRECISION
            dxl_state = dxl_state/[VEL_PRECISION, POS_PRECISION] - [0, 1]   
        return np.concatenate((dxl_load, dxl_state), axis=1), dxl_error, dxl_temp, fail


    def moveToStart(self):
        for j in range(self.num_reset_steps):
            goal_position = self.reset_steps[:,j]
                
            for i in range(self.num_servos):
                DXL_ID = self.ids[i]
                # Allocate goal position value into byte array
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position[i])), DXL_HIBYTE(DXL_LOWORD(goal_position[i])), DXL_LOBYTE(DXL_HIWORD(goal_position[i])), DXL_HIBYTE(DXL_HIWORD(goal_position[i]))] 
                # Add values to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(DXL_ID, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d]groupSyncWrite addparam failed" % DXL_ID)
                    quit()

            # Syncwrite goal position
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                quit()
            # Clear syncwrite parameter storage
            self.groupSyncWrite.clearParam()
            time.sleep(0.5)
       
    def writeAction(self, action):

        if len(action) != self.num_servos:
            print("Mismatched action size")
            quit()
        else:
            target = (np.array(action) + 1) / 2
     
            # convert actions to encoder positions
            goal_position = (target*(self.limits[:,1] - self.limits[:,0]) + self.limits[:,0]).astype(int)
          
        for i in range(self.num_servos):
            DXL_ID = self.ids[i]
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position[i])), DXL_HIBYTE(DXL_LOWORD(goal_position[i])), DXL_LOBYTE(DXL_HIWORD(goal_position[i])), DXL_HIBYTE(DXL_HIWORD(goal_position[i]))]

            # Add values to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(DXL_ID, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d]groupSyncWrite addparam failed" % DXL_ID)
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()


    def reboot(self, DXL_ID):
        self.setLED(DXL_ID, LED_OFF)
        self.setTorque(DXL_ID, TORQUE_DISABLE)  
        time.sleep(0.1)
    
        status = 0
        dxl_comm_result, _ = self.packetHandler.reboot(self.portHandler, DXL_ID)
        time.sleep(0.5)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            status = 1
        else:
            self.setupServo(DXL_ID)
            time.sleep(0.1)

        self.setLED(DXL_ID, LED_ON)
        self.setTorque(DXL_ID, TORQUE_ENABLE) 
        time.sleep(0.1)
        return status
        

    def close(self):
        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()

        for i in range(self.num_servos):
            DXL_ID = self.ids[i]
            self.setTorque(DXL_ID, TORQUE_DISABLE)  
            self.setLED(DXL_ID, LED_OFF)

        # Close port
        self.portHandler.closePort()

