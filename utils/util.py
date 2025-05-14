from enum import Enum
import time


class CommState(Enum):
    SYNC = 1
    COMM = 2


class ServiceType(Enum):
    CONTROL = 1
    TRAJECTORY = 2
    FORWARD_CONTROL = 3
    CONTROL_KOOPMAN = 4


class ControlFlag(Enum):
    SYNC_OK = b'\xA5'
    START = b'\xFF'


class ErrorFlag(Enum):
    BAD_CRC = 0x00
    NO_DATA_RECEIVED = 0xFE



def current_milli_time() -> int:
    """
    Returns the current time in milliseconds.

    This function retrieves the current time using the `time.time()` function,
    which returns the time in seconds since the epoch, and converts it to
    milliseconds by multiplying by 1000 and rounding to the nearest integer.

    Returns:
        int: The current time in milliseconds.
    """
    return round(time.time() * 1000)