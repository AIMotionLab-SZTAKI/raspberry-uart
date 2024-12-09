from interfaces.packet_handler_interface import PacketHandlerInterface
from datatypes.stabilizer_types import *
import struct
from typing import Optional, Tuple
from utils.debug import DEBUG_PRINT_CONTROL

class ControlPacketHandler(PacketHandlerInterface):


    """
    @staticmethod
    def packet_composition(thrust: float, attitude_rate: Attitude_t, data: bytes):
        data_new = struct.pack('<fifff', thrust, attitude_rate.timestamp, attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        #print(attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        data_new += data[len(data_new):]
        return data_new
    
    """
    
    @staticmethod
    def packet_composition(thrust: float, attitude_rate: Attitude_t) -> bytes:
        data = struct.pack('<fifff', thrust, attitude_rate.timestamp, attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        return data
    

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[Tuple[float, State_t]]:

        if payload_length % 4 != 0:
            return
        
        values = [data[i : i+4] for i in range(0, payload_length, 4)]
        #print(values)
        ## Thrust
        thrust: float = struct.unpack('<f',values[0])[0] # thrust

        ## Attitude
        attitude: Attitude_t = Attitude_t(
            struct.unpack('<i', values[1])[0], # timestamp
            struct.unpack('<f', values[2])[0], # row 
            struct.unpack('<f', values[3])[0], # pitch
            struct.unpack('<f', values[4])[0]  # yaw
        )

        ## AttitudeQuaternion
        quaternion: Quaternion_t = Quaternion_t()

        ## x, y, z, w coordinates
        orient_x: X = X(
            struct.unpack('<f', values[5])[0], # x
            struct.unpack('<f', values[6])[0], # y
            struct.unpack('<f', values[7])[0], # z
            struct.unpack('<f', values[8])[0], # w
        )
        quaternion.orient_x = orient_x

        ## Position
        position: Point_t = Point_t(
            struct.unpack('<i', values[9])[0],  # timestamp
            struct.unpack('<f', values[10])[0], # x
            struct.unpack('<f', values[11])[0], # y
            struct.unpack('<f', values[12])[0], # z
        )

        ## Velocity
        velocity: Velocity_t = Velocity_t(
            struct.unpack('<i', values[13])[0], # timestamp
            struct.unpack('<f', values[14])[0], # x
            struct.unpack('<f', values[15])[0], # y
            struct.unpack('<f', values[16])[0], # z
        )

        ## Acceleration
        acc: Acc_t = Acc_t(
            struct.unpack('<i', values[17])[0], # timestamp
            struct.unpack('<f', values[18])[0], # x
            struct.unpack('<f', values[19])[0], # y
            struct.unpack('<f', values[20])[0], # z
        )

        ## State
        state: State_t = State_t(
            attitude,
            quaternion,
            position,
            velocity,
            acc
        )

        # DEBUG_PRINT_CONTROL(thrust, state)

        return (thrust, state)
    
  

