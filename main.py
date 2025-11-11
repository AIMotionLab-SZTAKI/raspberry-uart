from raspberry_uart.uart.uart_serial import UARTCommunication
from raspberry_uart.utils.config import SERIAL_URI, BAUD_RATE
from raspberry_uart.utils.util import ControlFlag


if __name__ == "__main__":
    # SERIAL_URI and BAUD_RATE can be modified in utils/config.py
    
    comm = UARTCommunication(SERIAL_URI, BAUD_RATE)
    comm.communicate(None, None)
    comm.close_commmunication()
