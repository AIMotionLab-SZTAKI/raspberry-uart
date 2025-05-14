from interfaces.packet_handler_interface import PacketHandlerInterface
from datatypes.stabilizer_types import *
import struct
from typing import Optional, Tuple
from utils.debug import DEBUG_PRINT_CONTROL
import numpy as np

class KoopmanPacketHandler(PacketHandlerInterface):


    """
    @staticmethod
    def packet_composition(thrust: float, attitude_rate: Attitude_t, data: bytes):
        data_new = struct.pack('<fifff', thrust, attitude_rate.timestamp, attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        #print(attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        data_new += data[len(data_new):]
        return data_new
    
    """
    
    @staticmethod
    def packet_composition(thrust: float, torque_x: float, torque_y: float, torque_z: float) -> bytes:
        data = struct.pack('<ffff', thrust, torque_x, torque_y, torque_z)
        return data
    

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int):

        if payload_length % 4 != 0:
            return
        
        values = [data[i : i+4] for i in range(0, payload_length, 4)]

        state = np.zeros(12)

        state[0] = struct.unpack('<f', values[0])[0]
        state[1] = struct.unpack('<f', values[1])[0]
        state[2] = struct.unpack('<f', values[2])[0]
        state[3] = struct.unpack('<f', values[3])[0]
        state[4] = struct.unpack('<f', values[4])[0]
        state[5] = struct.unpack('<f', values[5])[0]
        state[6] = struct.unpack('<f', values[6])[0]
        state[7] = struct.unpack('<f', values[7])[0]
        state[8] = struct.unpack('<f', values[8])[0]
        state[9] = struct.unpack('<f', values[9])[0]
        state[10] = struct.unpack('<f', values[10])[0]
        state[11] = struct.unpack('<f', values[11])[0]

        print(state)

        return state
    
  

