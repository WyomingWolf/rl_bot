################################################################################
# Author: James Mock
# Date: 2021-08-12
#
# ******************* Dynamixel X-Series Servo Controller **********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

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
ADDR_PRESENT_LOAD		= 126
LEN_PRESENT_LOAD		= 2	    # Data Byte Length
ADDR_PRESENT_VELOCITY		= 128
LEN_PRESENT_VELOCITY		= 4	    # Data Byte Length         
ADDR_PRESENT_POSITION           = 132
LEN_PRESENT_POSITION            = 4         # Data Byte Lengt
ADDR_INDIRECTADDRESS_FOR_READ   = 168
LEN_INDIRECTDATA_FOR_READ       = 11        # Sum of Data of Length. i.e) Hardware Error (1byte) + Present Load (2 byte) + Present Velocity (4 byte) + Present Position (4 bytes)
ADDR_INDIRECTDATA_FOR_READ      = 224

TORQUE_DISABLE                  = 0         # Value for disabling the torque
TORQUE_ENABLE                   = 1         # Value for enabling the torque
LED_OFF                         = 0         # Dynamixel LED will light between this value
LED_ON                          = 1         # and this value

LOAD_PRECISION                  = 1000
VEL_PRECISION                   = 1023
POS_PRECISION                   = 4095



class ServoController:
    def __init__(self, servos):
        self.servos = servos
        self.num_servos = np.shape(self.servos)[0]

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
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTDATA_FOR_READ, fast=True)

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

        self.reboot(np.ones(self.num_servos)) # reboot all servos

        for i in range(self.num_servos):
            DXL_ID = self.servos[i,0]
            # Disable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Turn off LED
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_LED_RED, LED_OFF)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # INDIRECTDATA parameter storages replace present load, present velocity and present position
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 0, ADDR_HARDWARE_ERROR)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRESENT_LOAD + 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRESENT_LOAD + 1)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRESENT_VELOCITY + 0)
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
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_INDIRECTADDRESS_FOR_READ + 14, ADDR_PRESENT_POSITION + 0)
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

            # Enable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
   
            # Turn on LED
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_LED_RED, LED_ON)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            


            # Add parameter storage for multiple values
            dxl_addparam_result = self.groupSyncRead.addParam(DXL_ID)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID)
                quit()
            else:
                print("Dynamixel#%d: Online" % DXL_ID)


    def readState(self):
        #print("Read")
        dxl_error = np.zeros((self.num_servos,1), dtype=np.int8)
        dxl_load = np.zeros((self.num_servos,1), dtype=np.int16)	
        dxl_state = np.zeros((self.num_servos,2), dtype=np.int32)
        # Syncread present position from indirectdata
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        for i in range(self.num_servos):
            DXL_ID = self.servos[i,0]
                       
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

            # Get Dynamixel present error status
            dxl_error[i] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ, LEN_HARDWARE_ERROR)

            # Get Dynamixel present load value
            dxl_load[i] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR, LEN_PRESENT_LOAD)

            # Get Dynamixel present position value
            dxl_state[i,0] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD, LEN_PRESENT_VELOCITY)

            # Get Dynamixel present position value
            dxl_state[i,1] = self.groupSyncRead.getData(DXL_ID, ADDR_INDIRECTDATA_FOR_READ + LEN_HARDWARE_ERROR + LEN_PRESENT_LOAD + LEN_PRESENT_VELOCITY, LEN_PRESENT_POSITION)

        return np.concatenate((dxl_load, dxl_state), axis=1)/[LOAD_PRECISION, VEL_PRECISION, POS_PRECISION], dxl_error


    def moveToStart(self):
        goal_position = self.servos[:,1]
             

        for i in range(self.num_servos):
            DXL_ID = self.servos[i,0]
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
       
    def writeAction(self, action):
        #print(len(action))
        if len(action) != self.num_servos:
            print("Mismatched action size")
            quit()
        else:
            # convert actions to encoder positions
            goal_position = (action*(self.servos[:,3] - self.servos[:,2]) + self.servos[:,2]).astype(int)
          
        for i in range(self.num_servos):
            DXL_ID = self.servos[i,0]
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

    def reboot(self, errors):
         for i in range(self.num_servos):
            if errors[i] == 1:
                DXL_ID = self.servos[i,0]
                dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, DXL_ID)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def close(self):
        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()

        for i in range(self.num_servos):
            DXL_ID = self.servos[i,0]
            # Disable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                 print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                 print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Turn off LED
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_LED_RED, LED_OFF)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()

