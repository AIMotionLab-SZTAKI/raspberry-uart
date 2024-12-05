import serial
import struct
from datatypes.stabilizer_types import *
from typing import Tuple, Optional, Union
from datatypes.communication_packets import create_uart_packet, MAX_PAYLOAD_LENGTH
import traceback
from utils.util import CommState, ServiceType, ErrorFlag, ControlFlag, current_milli_time
from utils.config import *
from packet_handlers.control_packet_handler import ControlPacketHandler
from packet_handlers.trajectory_packet_handler import TrajectoryPacketHandler
from packet_handlers.forward_packet_handler import ForwardPacketHandler
from controllers.pid_control import PidControl
from ctypes import sizeof
from multiprocessing import shared_memory
import numpy as np
import time


class UARTCommunication:
    """
    A class to handle UART communication with a device.

    Attributes:
        ser (serial.Serial): The serial connection object.
        comm_state (CommState): The current communication state (SYNC or COMM).
        shutdown_transport (bool): Flag to indicate if the transport should be shut down.

    Methods:
        __init__(serial_port: str, baudrate: int):
            Initializes the UARTCommunication instance with the specified serial port and baud rate.
        
        communicate():
            Manages the communication process, handling synchronization and data transmission.
        
        close_commmunication():
            Closes the serial communication.
    """


    def __init__(self, serial_port: str, baudrate: int):
        self.ser: serial.Serial = self._open_serial_connection(serial_port, baudrate)
        self.comm_state: CommState = CommState.SYNC
        self.shutdown_transport: bool = False
        self.controller = PidControl()


    def communicate(self, shm_name, lock):
        self.shm = shared_memory.SharedMemory(name=shm_name)
        self.lock = lock
        try:
            while self.shutdown_transport is False:

                if self.comm_state == CommState.SYNC:
                    self._synchronize()
                
                while self.comm_state == CommState.COMM:

                    start_time = current_milli_time()
                    print(f'\n{SEPARATOR}')

                    ## Read the first byte of the packet
                    start = self.ser.read(1)

                    if start == ControlFlag.START.value:
                        print(f'Start: {start.hex()}')

                        ## Read service type and check if its correct
                        try:
                            service_type = int(self.ser.read(1).hex(), 16)
                            payloadLength = int(self.ser.read(1).hex(), 16)
                        except ValueError as e:
                            print(e)
                            self.reset_communication()
                    else:
                        print(f'Start is not 0xFF')                 
                        self.reset_communication()
                        

                    if self.comm_state == CommState.COMM and payloadLength > 0:
                        self._handle_packet(payloadLength, service_type)

                                       
                        print(f'\nTransaction time in ms: {current_milli_time() - start_time}')
                        print(f'{SEPARATOR}\n')
                        
                    else:
                        self.reset_communication()


        except Exception as e:
            print(f'Error reading from serial port: {e}')
            traceback.print_exc()



    def reset_communication(self):
        self.comm_state = CommState.SYNC



    def close_commmunication(self) -> None:
        self.shutdown_transport = True
        self.ser.close()




    def _synchronize(self) -> None:
        """
        Synchronizes the communication with the device.

        This function waits for a synchronization byte from the device and, upon
        receiving the correct byte, sets the communication state to COMM. It also
        configures custom timeouts for read and write operations during the communication
        state and sends a confirmation byte back to the device.

        Steps:
        1. Resets the serial input and output buffers.
        2. Waits for a synchronization byte from the device.
        3. If the correct sync byte is received, sets the communication state to COMM.
        4. Configures custom timeouts for read and write operations.
        5. Sends a confirmation byte (0xA5) back to the device.

        Prints the status of the synchronization process.
        """

        self.ser.timeout = None
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        print("Waiting for sync bytes...")
        
        sync_buffer = self.ser.read(1)
        print(f'Sync byte: {sync_buffer.hex()}')

        if sync_buffer == ControlFlag.SYNC_OK.value:
            self.comm_state = CommState.COMM
            
            ## Set custom timeout for read and write ops
            ## during comm state
            self.ser.timeout = READ_TIMEOUT
            self.ser.write_timeout = WRITE_TIMEOUT

            ## Send back flag SYNC_OK (byte 0xA5), that indicates successful synchronization
            self._send_flag(ControlFlag.SYNC_OK)





    def _open_serial_connection(self, serial_port: str, baudrate: int) -> serial.Serial:
        """
        Opens a serial connection to the specified port with the given baud rate.

        This function attempts to open a serial connection using the provided
        serial port and baud rate. If the connection is successfully opened,
        it returns the serial object. If the connection fails, it prints an
        error message and exits the program.

        Args:
            serial_port (str): The name of the serial port to connect to.
            baudrate (int): The baud rate for the serial connection.

        Returns:
            serial.Serial: The opened serial connection object.

        Raises:
            Exception: If an error occurs while opening the serial port.
        """

        try:
            ser = serial.Serial(serial_port, baudrate=baudrate)
            if ser.is_open:
                print('Serial port opened successfully.')
                return ser
            else:
                print('Failed to open serial port.')
        except Exception as e:
            print(f'Error opening serial port: {e}')
            traceback.print_exc()
            exit(1)


    


    def _calc_crc(self, data: bytes) -> int:
        """
        Calculates the CRC (Cyclic Redundancy Check) for the given data.

        This function computes the CRC by performing an XOR operation on each byte
        of the input data.

        Args:
            data (bytes): The input data for which the CRC is to be calculated.

        Returns:
            int: The calculated CRC value as an integer.
        """

        crc: int = 0
        for p in data:
            crc ^= p
        return crc
    



    def _handle_crc(self, crc_value: int, data: bytes) -> bool:
        """
        Handles the CRC (Cyclic Redundancy Check) validation for the given data.

        This function calculates the CRC for the provided data and compares it
        with the given CRC value to determine if they match.

        Args:
            crc_value (int): The expected CRC value to compare against.
            data (bytes): The input data for which the CRC is to be calculated.

        Returns:
            bool: True if the calculated CRC matches the given CRC value, False otherwise.
        """

        calculated_crc = self._calc_crc(data)
        
        print(f'CRC {crc_value} - {calculated_crc} Calculated CRC')
        return calculated_crc == crc_value 




    def _send_flag(self, flag: Union[ControlFlag, ErrorFlag]) -> None:
        """
        Sends a flag to the serial communication interface.

        Args:
            flag (Union[ControlFlag, ErrorFlag]): The flag to be sent. Must be an instance of either ControlFlag or ErrorFlag.

        Raises:
            TypeError: If the flag is not an instance of ControlFlag or ErrorFlag.

        Description:
            This method sends a flag to the serial communication interface. The flag can be either a ControlFlag or an ErrorFlag.
            The flag's value is packed into a byte and written to the serial interface. If the flag is an ErrorFlag, the communication
            is reset and an error message is printed. Otherwise, a communication message is printed.

        Example:
            _send_flag(ControlFlag.START)
            _send_flag(ErrorFlag.TIMEOUT)
        """
        if not isinstance(flag, (ControlFlag, ErrorFlag)):
            raise TypeError(f"Expected ControlFlag or ErrorFlag, but got {type(flag).__name__}")  ## Check if the flag is either a ControlFlag or ErrorFlag
        

        packet = b''
        packet += struct.pack('B', int(flag.value.hex(), 16)) ## Flags are stored in hex, but struct.pack requires int
        self.ser.write(packet)

        if isinstance(flag, ErrorFlag): ## If the flag is an ErrorFlag reset the communication
            self.reset_communication() 
            print(f'An error occured during communication: {flag.name}')
        else:
            print(f'COMM [{flag.name}]')




    def _send_packet(self, data: bytes, service_type: int) -> None:

        if len(data) > MAX_PAYLOAD_LENGTH: ## If there are more data than the MAX_PAYLOAD_LENGTH reset communication.
            print(f"Payload size is bigger than {MAX_PAYLOAD_LENGTH}!")
            self.reset_communication()
        
        packet = create_uart_packet(len(data))  ## This functions creates a UARTPacket object with a payload size of len(data)
        print('Packet size: ', sizeof(packet))
        print('Payload size: ', sizeof(packet.payload))
        packet.start = 0xFF
        packet.serviceType = service_type
        packet.payloadLength = len(data)

        for i, byte in enumerate(data):
            packet.payload[i] = byte

        packet.crc = self._calc_crc(data)
        print("Sending bytes...")
        
        self.ser.write(packet)


    def _handle_packet(self, payloadLength: int, service_type: int) -> Optional[Tuple[float, State_t]]:

        print(f'Useful payload length in bytes: {payloadLength}')

        data = self.ser.read(payloadLength)
        
        ## Check if the read data has the same length as expected
        ## If its not then a timeout
        if len(data) == payloadLength:
            
            if service_type == ServiceType.CONTROL.value:    
                print('Service type: CONTROL')

                _, state = ControlPacketHandler.packet_decomposition(data, payloadLength)
                setpoint = Setpoint_t(Attitude_t(0, 0, 0, 0),
                                      Attitude_t(0, 0, 0, 0),
                                      Quaternion_t(Q(0, 0, 0, 1), X(0, 0, 0, 1)),
                                      0,
                                      Point_t(0, 0, 0, 0.3),
                                      Velocity_t(0, 0, 0, 0),
                                      Acc_t(0, 0, 0, 0))

                thrust, attitude_rate = self.controller.position_controller(setpoint, state)               

            elif service_type == ServiceType.TRAJECTORY.value:
                print('Service type: TRAJECTORY')
                TrajectoryPacketHandler.packet_decomposition(data, payloadLength)

            elif service_type == ServiceType.FORWARD_CONTROL.value:
                print('Service type: FORWARD_CONTROL')
                ForwardPacketHandler.packet_decomposition(data, payloadLength)

            else:
                print("Service type unknown.")
                self.reset_communication()

            try:
                crc = int(self.ser.read(1).hex(), 16)
            except ValueError as e:
                print(e)
                self.reset_communication()
                return
            
            ## If crc is correct return data
            ## Else return wrong crc flag (0xFE)
            if self._handle_crc(crc_value = crc, data = data):
                if service_type == ServiceType.CONTROL.value: 
                    data = ControlPacketHandler.packet_composition(thrust, attitude_rate)

                elif service_type == ServiceType.TRAJECTORY.value:
                    data = TrajectoryPacketHandler.packet_composition(88.88, 99.99) ## creates dummy data and sends it back

                elif service_type == ServiceType.FORWARD_CONTROL.value:
                    data_in = np.zeros(5)
                    time.sleep(0.0001)
                    with self.lock:
                        shared_array = np.ndarray((5, ), dtype=np.float64, buffer=self.shm.buf)
                        np.copyto(data_in, shared_array)
                    data = ForwardPacketHandler.packet_composition(data_in)
                
                self._send_packet(data, service_type)

            else:
                self._send_flag(ErrorFlag.BAD_CRC) ## If the CRC is incorrect send a BAD_CRC back
        else:
            self._send_flag(ErrorFlag.NO_DATA_RECEIVED) ## If the payloadLength is not equal to the payload size, send back a NO_DATA_RECEIVED flag

