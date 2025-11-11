from ctypes import Structure, c_byte, sizeof
from raspberry_uart.utils.config import MAX_PAYLOAD_LENGTH

"""
class UARTPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ('start', c_byte),
        ('serviceType', c_byte),
        ('payloadLength', c_byte),
        ('payload', c_byte * MAX_PAYLOAD_LENGTH),
        ('crc', c_byte)
    ]
"""

def create_uart_packet(max_payload_length: int):
    class UARTPacket(Structure):
        _pack_ = 1
        _fields_ = [
            ('start', c_byte),
            ('serviceType', c_byte),
            ('payloadLength', c_byte),
            ('payload', c_byte * max_payload_length),
            ('crc', c_byte)
        ]
    return UARTPacket()