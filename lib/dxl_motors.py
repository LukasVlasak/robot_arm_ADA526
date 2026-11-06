import dynamixel_sdk as dxl
import numpy as np
from adatools import utils
import time

# This example shows how to use the Dynamixel SDK send and receive joint positions.
# The example uses the Dynamixel SDK's GroupSyncWrite and GroupSyncRead functions to send and receive data to and from the servos.

class DXLMotors:
    def __init__(self,dxl_ids,port='/dev/ttyUSB0', baudrate=57600):
        # Define some constants
        self.addr_torque_enable          = 64
        self.addr_goal_position          = 116
        self.len_goal_position           = 4         # data byte length
        self.addr_present_position       = 132
        self.len_present_position        = 4         # data byte length
        self.baudrate                    = baudrate
        self.addr_is_moving              = 122
        self.protocol_version            = 2.0
        self.addr_operating_mode         = 11
        self.extended_position_val       = 4
        self.addr_profile_velocity       = 112  
        self.len_profile_velocity        = 4
        self.port                        = port  # <-- you may need to change the port where the u2d2 is connected
        self.dxl_ids                     = dxl_ids  # [1, 2, 3, 4, 5]

        self.__init()

    def __init(self):
        # init port & packet hanler
        self.portHandler = dxl.PortHandler(self.port)
        self.packetHandler = dxl.PacketHandler(self.protocol_version) 

        # rdy to communicate
        self.__open_port()
        self.__setBaudRate(self.baudrate)
        
        # group readers/writers
        self.presentPositionReader = dxl.GroupSyncRead(self.portHandler, self.packetHandler, self.addr_present_position, self.len_present_position)
        self.goalPositionWriter = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, self.addr_goal_position, self.len_goal_position)
        self.profileVelocityWriter = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, self.addr_profile_velocity, self.len_profile_velocity)
        self.operatingModeWriter = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, self.addr_operating_mode, 1)

        # initial settings
        self.__setExtendedOperatingMode()
        self.setDXLTorqueEnabled(True)
        self.setMotorsSpeed(45)

    def __open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:  
            print("Failed to open the port")
            quit()

    def __setBaudRate(self, baudrate):
        if self.portHandler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    # kvuli tem limitam a DH modelu ...
    def __setExtendedOperatingMode(self):
        self.operatingModeWriter.clearParam()

        for id in self.dxl_ids:
            self.operatingModeWriter.addParam(id, [self.extended_position_val])

        dxl_comm_result = self.operatingModeWriter.txPacket()
        self.__checkRWResult(dxl_comm_result)

        # 4. ČIŠTĚNÍ PO ODESLÁNÍ (Doporučená praxe pro vyčištění paměti)
        self.operatingModeWriter.clearParam()

    def __toSigned32(self, v):
        return v - (1 << 32) if v >= (1 << 31) else v

    def setDXLTorqueEnabled(self, enable):
        for id in self.dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.addr_torque_enable, 1 if enable == True else 0)
            self.__checkRWResult(dxl_comm_result, dxl_error, "Motor " + str(id) + ": Torque enabled")



    def getIsMoving(self, id):
        self.__checkID(id)

        is_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, self.addr_is_moving)

        self.__checkRWResult(dxl_comm_result, dxl_error)
        return is_moving == 1

    def getSingleMotorPos(self, id):
        self.__checkID(id)
        
        position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.addr_present_position)
        self.__checkRWResult(dxl_comm_result, dxl_error)

        return self.__toSigned32(position)
    
    def getMotorsPos(self, ids=None):
        result = []
        if ids is None:
            ids = self.dxl_ids

        self.presentPositionReader.clearParam()
        for id in ids:
            self.presentPositionReader.addParam(id)

        dxl_comm_result = self.presentPositionReader.txRxPacket()
        self.__checkRWResult(dxl_comm_result)

        for id in ids:
            is_available = self.presentPositionReader.isAvailable(id, self.addr_present_position, self.len_present_position)

            if not is_available:
                raise RuntimeError("Data for group reading not available")

            pos = self.presentPositionReader.getData(id, self.addr_present_position, self.len_present_position)
            result.append(self.__toSigned32(pos))

        return result

    def setMotorsGoalPos(self, pos):
        """
            Pri zadani -1 motor zustane a ostatni se pohnou
            Funguje i pri zadani mene motoru - napr. 3 kdyz mam 5 - tak proste vezme prvni 3
        """
        if len(pos) > len(self.dxl_ids) or len(pos) == 0:
            print("DEMENTE TOLIK MOTORU NEMAS")
            return

        self.goalPositionWriter.clearParam()
        goal_pos = utils.to4bytes(pos)

        for i in range(len(goal_pos)):
            if int(pos[i]) != -1:
                self.goalPositionWriter.addParam(self.dxl_ids[i], goal_pos[i])

        dxl_comm_result = self.goalPositionWriter.txPacket()
        self.__checkRWResult(dxl_comm_result)

        # 4. ČIŠTĚNÍ PO ODESLÁNÍ (Doporučená praxe pro vyčištění paměti)
        self.goalPositionWriter.clearParam()

    def setMotorsSpeed(self, speed):
        """
            42 - 45 mega pomaly
            46-52 ty MAGOREEEEEE
        """
        self.profileVelocityWriter.clearParam()
        
        param_velocity_43 = utils.to4bytes(speed)[0]

        for dxl_id in self.dxl_ids:
            self.profileVelocityWriter.addParam(dxl_id, param_velocity_43)

        dxl_comm_result = self.profileVelocityWriter.txPacket()
        self.__checkRWResult(dxl_comm_result)

        self.profileVelocityWriter.clearParam()

    def setSingleMotorSpeed(self, id, speed):
        """
            42 - 45 mega pomaly
            46-52 ty MAGOREEEEEE
        """
        self.__checkID(id)
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.addr_profile_velocity, int(speed))
        self.__checkRWResult(dxl_comm_result, dxl_error)

    def setSingleMotorGoalPos(self, pos, id):
        self.__checkID(id)
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.addr_goal_position, int(pos))
        self.__checkRWResult(dxl_comm_result, dxl_error)



    def __checkID(self, id):
        if id not in self.dxl_ids:
            raise ValueError("ID is not in available IDs")

    def __checkRWResult(self, dxl_comm_result, dxl_error=None, success_msg=None):
        if dxl_comm_result != dxl.COMM_SUCCESS:
            err_msg = self.packetHandler.getTxRxResult(dxl_comm_result)
            raise ValueError(f"[COMM ERROR] {err_msg}")

        if dxl_error is not None and dxl_error != 0:
            err_msg = self.packetHandler.getRxPacketError(dxl_error)
            raise ValueError(f"[HARDWARE ERROR] {err_msg}")

        if success_msg is not None:
            print(success_msg)

    
    