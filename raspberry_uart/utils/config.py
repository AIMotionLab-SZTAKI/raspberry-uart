from typing import Optional


## Constrans for the serial interface
SERIAL_URI: str = '/dev/ttyAMA0'
BAUD_RATE: int = 921600
CPU_CORE_UART = 3
RT_PRIORITY = 90

READ_TIMEOUT: Optional[float] = 0.1
WRITE_TIMEOUT: Optional[float] = None


#SYNC_BYTE: bytes = b'\xA5'
#START_BYTE: bytes = b'\xFF'


## Separator for prints (for debug purposes)
SEPARATOR: str = "=" * 100


## Constants for communication packet
MAX_UART_BUFFER_SIZE = 200

PACKET_HEADER_LENGTH = 3
PACKET_CRC_LENGTH = 1
PACKET_META_LENGTH = (PACKET_HEADER_LENGTH + PACKET_CRC_LENGTH)

MAX_PAYLOAD_LENGTH = (MAX_UART_BUFFER_SIZE - PACKET_META_LENGTH)
