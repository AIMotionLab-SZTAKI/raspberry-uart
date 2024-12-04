from interfaces.packet_handler_interface import PacketHandlerInterface
import struct
from typing import List, Optional
from utils.debug import DEBUG_PRINT_TRAJECTORY


class TrajectoryPacketHandler(PacketHandlerInterface):

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[List[float]]:
        values = [data[i : i+4] for i in range(0, payload_length, 4)]
        dummy_list: List[float] = []
        dummy_list.append(struct.unpack('<f', values[0])[0])
        dummy_list.append(struct.unpack('<f',values[1])[0])
        dummy_list.append(struct.unpack('<f', values[2])[0])

        DEBUG_PRINT_TRAJECTORY(dummy_list)

        return dummy_list

    
    ## Placeholder method.
    ## To implement properly in the future, override this method and its parameters as needed.
    @staticmethod
    def packet_composition(traj_1, traj_2):
        data = struct.pack('<ff', traj_1, traj_2)
        return data
    