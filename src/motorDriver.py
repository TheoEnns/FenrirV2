import os, sys, time, math
from threading import Thread, Lock
os.sys.path.append('../dynamixel_functions_py')             # Path setting, dynamixel wants this for reasons unknown
from dynamixel_sdk import *                                 # Uses Dynamixel SDK library

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Port Control
BAUDRATE                    = 3000000                       #57600 115200 1000000 2000000 3000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Default setting
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

#Init checking
isPortOpen = False
isHandlerInit = False

#Threading Managment
interval_command = 0.01

class DynamixelMaster(Thread):
    
    ####
    # initialization
    ####

    def __init__(self):
        Thread.__init__(self)
        self.commandLock = Lock()
        self.goalLock = Lock()
        self.posLock = Lock()
        self.is_running = False
        self.daemon = False
        self._init_dyn_handler()

    def _init_dyn_handler(self):
        self.goalLock.acquire()
        self.commandLock.acquire()
        self.posLock.acquire()
        try:
            self.portHandler = port_handler.PortHandler(DEVICENAME)
            self.packetHandler = packet_handler.PacketHandler(PROTOCOL_VERSION)
            self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler,
                ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
            self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, 
                ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

            if self.portHandler.openPort():  
                print "Succeeded to open the port!"
            else:
                print "Failed to open the port!"
                return False

            # Set port baudrate
            if self.portHandler.setBaudRate( BAUDRATE):
                print "Succeeded to change the baudrate!"
            else:
                print "Failed to change the baudrate!"
                return False
        finally:
            self.goalLock.release()
            self.commandLock.release()
            self.posLock.release()

        self.current_pos = self.discover_servos()
        for ID in  self.current_pos:
            self.write_current(ID,2048)
        self.poll_current_positions()
        self.goal_pos = self.read_currents()
        return True

    def closeDynamixel(self):
        self.groupSyncRead.clearParam()
        self.portHandler.closePort()
    
    ####
    # Threadsafe Sync Commands
    ####

    def __bad__discover_servos(self, display=False):
        """ TODO: Debug why this only finds servos 5 and 6 not 32 to 43"""
        dxl_data_list, dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler)
        if dxl_comm_result != COMM_SUCCESS:
            print("discover_servos: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        if display: 
            print("Detected Dynamixel :")
            for dxl_id in dxl_data_list:
                print("[ID:%03d] model version : %d | firmware version : %d" %
                    (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))
        self.IDs = dxl_data_list.keys()
        return dxl_data_list

    def discover_servos(self, display=False):
        servos = {}
        self.IDs = []
        self.commandLock.acquire()
        try:
            self.groupSyncRead.clearParam()
            for ID in range(1,50):
                dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, ID)
                if dxl_comm_result != COMM_SUCCESS:
                    # print "%s" % packetHandler.getTxRxResult(dxl_comm_result) 
                    pass
                elif dxl_error != 0:
                    # print "%s" % packetHandler.getRxPacketError(dxl_error) 
                    pass
                else:
                    if display:
                        print "[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (ID, dxl_model_number)
                    self.IDs.append(ID)
                    servos[ID] = dxl_model_number
                    dxl_addparam_result = self.groupSyncRead.addParam(ID)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncRead addparam failed" % indx)
        finally:
            self.commandLock.release()
        for ID in self.IDs:
            self.torque_toggle(ID, True)
        return servos
                

    def poll_current_positions(self, display=False):
        self.commandLock.acquire()
        try:
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("update_currents: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            for ID in self.IDs:
                dxl_getdata_result = self.groupSyncRead.isAvailable(ID, 
                    ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % ID)
                    return False
            for ID in self.IDs:
                self.write_current(ID,self.groupSyncRead.getData(ID, 
                    ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION))
        finally:
            self.commandLock.release()
        return True

    def poll_current_position(self, ID, display=False):
        self.commandLock.acquire()
        try:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, ID, ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                return False
            self.write_current(ID,dxl_present_position)
            if display:    
                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (ID, self.read_goal(ID), self.read_current(ID)))
        finally:
            self.commandLock.release()
        return True

    def command_goals(self, display=False):
        goals = self.read_goals()
        self.commandLock.acquire()
        try:
            for ID in self.IDs:
                param_goal_position = [ DXL_LOBYTE(DXL_LOWORD(goals[ID])), 
                                        DXL_HIBYTE(DXL_LOWORD(goals[ID])), 
                                        DXL_LOBYTE(DXL_HIWORD(goals[ID])), 
                                        DXL_HIBYTE(DXL_HIWORD(goals[ID]))]
                dxl_addparam_result = self.groupSyncWrite.addParam(ID, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % ID)
                    return False
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            self.groupSyncWrite.clearParam()
        finally:
            self.commandLock.release()
        return True

    def command_goal(self, ID, goal, display=False):
        self.write_goal(ID,goal)
        self.commandLock.acquire()
        try:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID,
                ADDR_PRO_GOAL_POSITION, goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                return False
        finally:
            self.commandLock.release()
        return True

    def reboot_motors(self, display=False):
        # Need to find a broadcast method
        self.commandLock.acquire()
        try:
            self.groupSyncRead.clearParam()
        finally:
            self.commandLock.release()
        for ID in self.IDs:
            self.reboot_motor(ID, display)
            self.commandLock.acquire()
            try:
                dxl_addparam_result = self.groupSyncRead.addParam(ID)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncRead addparam failed" % indx)
                    return False
            finally:
                self.commandLock.release()
        return True

    def reboot_motor(self, ID, display=False):
        self.commandLock.acquire()
        try:
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, ID)
            if dxl_comm_result != COMM_SUCCESS:
                print("reboot_motor 1: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            elif dxl_error != 0:
                print("reboot_motor 2: %s" % self.packetHandler.getRxPacketError(dxl_error))
                return False
            if display: 
                print("[ID:%03d] reboot Succeeded\n" % DXL_ID)
        finally:
            self.commandLock.release()
        time.sleep(0.5)
        self.torque_toggle(ID, True)
        return True
        
    def torque_toggle(self, ID, state, display=False):
        if state:
            toggle = TORQUE_ENABLE
        else:
            toggle = TORQUE_DISABLE
        self.commandLock.acquire()
        try:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("torque_toggle 1: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return False
            elif dxl_error != 0:
                print("torque_toggle 2: %s" % self.packetHandler.getRxPacketError(dxl_error))
                return False
            if display: 
                print("[ID:%03d] reboot Succeeded\n" % ID)
        finally:
            self.commandLock.release()
        return True
    
    ####
    # Lifecycle Managment
    ####

    def __del__(self):
        self.closeDynamixel()

    def run(self):
        self.running = True
        
        # interval_command
        targetTime = time.time() + interval_command
        while(self.running):
            self.command_goals()
            elapseTime = targetTime - time.time()
            if elapseTime > 0:
                time.sleep(elapseTime) # Rest for a beat
                targetTime = targetTime + interval_command # Advance to the next beat
            elif elapseTime < 0:
                targetTime = time.time() + interval_command # Slip a beat
                print "Motor Update Underrun: ", str(elapseTime), " secs"
            else:
                targetTime = targetTime + interval_command # Advance to the next beat

    def stop(self):
        self.running = False
    
    ####
    # Thread Safe Accessors
    ####
    
    def read_goal(self,ID):
        self.goalLock.acquire()
        try:
            value = self.goal_pos[ID]
        finally:
            self.goalLock.release() 
        return value

    def read_goals(self):
        self.goalLock.acquire()
        try:
            values = self.goal_pos.copy()
        finally:
            self.goalLock.release() 
        return values

    def write_goal(self,ID,value):
        self.goalLock.acquire()
        try:
            self.goal_pos[ID] = value
        finally:
            self.goalLock.release() 
        return value
        
    def read_current(self,ID):
        self.posLock.acquire()
        try:
            value = self.current_pos[ID]
        finally:
            self.posLock.release()
        return value

    def read_currents(self):
        self.posLock.acquire()
        try:
            values = self.current_pos.copy()
        finally:
            self.posLock.release()
        return values

    def write_current(self,ID,value):
        self.posLock.acquire()
        try:
            self.current_pos[ID] = value
        finally:
            self.posLock.release()
        return value
