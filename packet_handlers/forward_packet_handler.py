from interfaces.packet_handler_interface import PacketHandlerInterface
import struct
from typing import List, Optional


class ForwardPacketHandler(PacketHandlerInterface):
    packet_size = 7

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[List[float]]:
        # maybe we will send back some info from the CF in the future 
        # and process it here, for now it is empty
        return None

    @staticmethod
    def packet_composition(data_in):
        data = struct.pack('<' + ForwardPacketHandler.packet_size * 'f', *data_in)
        return data
    