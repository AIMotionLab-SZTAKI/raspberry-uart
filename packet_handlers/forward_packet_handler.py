from interfaces.packet_handler_interface import PacketHandlerInterface
import struct
from typing import List, Optional
import json
import os

class ForwardPacketHandler(PacketHandlerInterface):
    packet_size = 7

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[List[float]]:
        pass  # don't do anything if not necessary, it just messes up communication sometimes
        # get PWM value from the CF in UINT16
        # if payload_length % 2 != 0:
        #     return
        
        # values = [data[i : i+2] for i in range(0, payload_length, 2)]
        #print(values)
        # pwm_0 = struct.unpack('<H',values[0])[0]
        # pwm_1 = struct.unpack('<H',values[1])[0]
        # pwm_2 = struct.unpack('<H',values[2])[0]
        # pwm_3 = struct.unpack('<H',values[3])[0]
        # pwm = [pwm_0, pwm_1, pwm_2, pwm_3]

        # save data
        # logfile = os.path.join(os.path.dirname(__file__), "..", "pwm_log.jsonl")
        # store as a dict with 2 entries

        # with open(logfile, "a") as f:   # append mode
        #     f.write(json.dumps(pwm) + "\n")
        # print(pwm)


    @staticmethod
    def packet_composition(data_in):
        data = struct.pack('<' + ForwardPacketHandler.packet_size * 'f', *data_in)
        return data
    