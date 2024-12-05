from interfaces.packet_handler_interface import PacketHandlerInterface
import struct
from typing import List, Optional


class ForwardPacketHandler(PacketHandlerInterface):

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[List[float]]:
        # maybe we will send back some info from the CF in the future 
        # and process it here, for now it is empty
        return None

    @staticmethod
    def packet_composition(data_in):
        data = struct.pack('<fffff', *data_in)
        return data
    