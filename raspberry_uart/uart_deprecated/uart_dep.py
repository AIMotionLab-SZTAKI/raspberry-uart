import serial
import struct
import time
from raspberry_uart.datatypes.stabilizer_types import *
from typing import Tuple, Optional, List
from enum import Enum
from raspberry_uart.datatypes.communication_packets import UARTPacket, MAX_PAYLOAD_LENGTH
import traceback


class CommState(Enum):
    SYNC = 1
    COMM = 2


class ServiceType(Enum):
    CONTROL = 1
    TRAJECTORY = 2


# Separator for prints
SEPARATOR: str = "=" * 100

SERIAL_URI: str = '/dev/tty.usbserial-1110'
BAUD_RATE: int = 500000

READ_TIMEOUT: Optional[float] = 0.01
WRITE_TIMEOUT: Optional[float] = None

SYNC_BYTE: bytes = b'\xA5'
START_BYTE: bytes = b'\xFF'



class UARTCommunication():
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
        
        # interface checks


    def communicate(self):
        """
        Manages the communication loop with the device over the serial connection.

        This function continuously checks the communication state and handles
        synchronization and data packet processing. It reads packets from the
        serial connection, identifies the service type, and processes the payload
        accordingly. The function also handles exceptions and prints debug information.

        The communication loop performs the following steps:
        1. Checks if the communication state is SYNC and attempts to synchronize.
        2. If synchronized, enters the COMM state and starts processing packets.
        3. Reads the start byte of the packet and verifies it.
        4. Reads the service type and payload length.
        5. Processes the payload based on the service type (CONTROL or TRAJECTORY, etc...).
        6. Prints transaction time and debug information.
        7. Handles exceptions and prints error messages.

        The loop continues until `shutdown_transport` is set to True.

        Raises:
            Exception: If an error occurs while reading from the serial port.
        """
        try:
            while self.shutdown_transport is False:

                if self.comm_state == CommState.SYNC:
                    self._synchronize()
                
                while self.comm_state == CommState.COMM:
                    start_time = self._current_milli_time()
                    print(f'\n{SEPARATOR}')

                    ## Read the first byte of the packet
                    start = self.ser.read(1)
                    
                    if start == START_BYTE:
                        print(f'Start: {start.hex()}')

                        ## Read service type and check if its correct
                        try:
                            service_type = int(self.ser.read(1).hex(), 16)
                            payloadLength = int(self.ser.read(1).hex(), 16)
                        except ValueError as e:
                            print(e)
                            self.comm_state = CommState.SYNC
                    else:
                        print(f'Start is not 0xFF')                 
                        self.comm_state = CommState.SYNC
                        

                    if self.comm_state == CommState.COMM and payloadLength > 0:

                        if service_type == ServiceType.CONTROL.value:    
                            print('Service type: CONTROL')
                            self._handle_control_packet(payloadLength)

                        elif service_type == ServiceType.TRAJECTORY.value:
                            print('Service type: TRAJECTORY')
                            self._handle_trajectory_packet(payloadLength)

                        else:
                            print("Service type unknown.")
                                       
                        print(f'\nTransaction time in ms: {self._current_milli_time() - start_time}')
                        print(f'{SEPARATOR}\n')

        except Exception as e:
            print(f'Error reading from serial port: {e}')
            traceback.print_exc()


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

        sync_buffer = b'\x00'
        print("Waiting for sync bytes...")
        
        sync_buffer = self.ser.read(1)
        print(f'Sync byte: {sync_buffer.hex()}')

        if sync_buffer == SYNC_BYTE:
            self.comm_state = CommState.COMM
            
            ## Set custom timeout for read and write ops
            ## during comm state
            self.ser.timeout = READ_TIMEOUT
            self.ser.write_timeout = WRITE_TIMEOUT

            ## Send back byte 0xA5, that indicates successful synchronization
            packet = b''
            packet += struct.pack('B', 0xA5)

            self.ser.write(packet)
            print("COMM [OK]")



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



    def _send_error(self) -> None:
        """
        Sends an error packet and resets the communication state.

        This function constructs an error packet with a predefined byte (0x00)
        and sends it over the serial connection. It then resets the communication
        state to SYNC and prints a message indicating a bad CRC.

        The error packet is used to signal an issue, such as a CRC mismatch, to the
        connected device.

        Returns:
            None
        """
        packet = b''
        packet += struct.pack('B', 0x00)

        self.ser.write(packet)
        self.comm_state = CommState.SYNC
        print("Bad CRC.")


    def _send_packet(self, data: bytes, service_type: int) -> None:
        """
        Sends a data packet over the serial connection.

        This function constructs a UART packet with the provided data, calculates
        the CRC for the data, and sends the packet over the serial connection.

        Args:
            data (bytes): The data to be sent in the packet.

        The packet structure includes:
        - A start byte (0xFF).
        - A service type (0x01).
        - The length of the payload.
        - The payload data.
        - The CRC of the payload data.

        Prints a message indicating that the bytes are being sent.
        """
        packet = UARTPacket()
        packet.start = 0xFF
        packet.serviceType = service_type
        packet.payloadLength = len(data)
        packet.payload[:packet.payloadLength] = data
        #packet.payload[len(data)] = self._calc_crc(data)
        
        packet.crc = self._calc_crc(data)
        print("Sending bytes...")
        self.ser.write(packet)



    def _current_milli_time(self) -> int:
        """
        Returns the current time in milliseconds.

        This function retrieves the current time using the `time.time()` function,
        which returns the time in seconds since the epoch, and converts it to
        milliseconds by multiplying by 1000 and rounding to the nearest integer.

        Returns:
            int: The current time in milliseconds.
        """
        return round(time.time() * 1000)


    ## A hardcoded decomposition method for the control packet's payload
    def _control_payload_decomp_static(self, data: bytes, payload_length: int) -> Optional[Tuple[float, State_t]]:
        """
        Decomposes the control payload into its constituent components.

        This function takes a byte array representing the control payload and its length,
        and decomposes it into various components such as thrust, attitude, quaternion,
        position, velocity, and acceleration. It then constructs a State_t object from
        these components and returns it along with the thrust value.

        Args:
            data (bytes): The byte array containing the control payload.
            payload_length (int): The length of the payload in bytes.

        Returns:
            Optional[Tuple[float, State_t]]: A tuple containing the thrust value and the
            decomposed state if the payload length is valid, otherwise None.

        The function performs the following steps:
        1. Checks if the payload length is a multiple of 4.
        2. Splits the data into 4-byte chunks and converts them to hexadecimal strings.
        3. Unpacks the hexadecimal strings into their respective components.
        4. Constructs the thrust, attitude, quaternion, position, velocity, and acceleration objects.
        5. Constructs the state object from the decomposed components.
        6. Prints the thrust and state for debugging purposes.
        7. Returns the thrust and state as a tuple.
        """

        if payload_length % 4 != 0:
            return
        
        values = [data[i : i+4].hex() for i in range(0, payload_length, 4)]
        #print(values)
        ## Thrust
        thrust: float = struct.unpack('<f', bytes.fromhex(values[0]))[0]

        ## Attitude
        attitude: Attitude_t = Attitude_t(
            struct.unpack('<i', bytes.fromhex(values[1]))[0],
            struct.unpack('<f', bytes.fromhex(values[2]))[0],
            struct.unpack('<f', bytes.fromhex(values[3]))[0],
            struct.unpack('<f', bytes.fromhex(values[4]))[0]
        )

        ## AttitudeQuaternion
        quaternion: Quaternion_t = Quaternion_t()

        ## x, y, z, w coordinates
        orient_x: X = X(
            struct.unpack('<f', bytes.fromhex(values[5]))[0],
            struct.unpack('<f', bytes.fromhex(values[6]))[0],
            struct.unpack('<f', bytes.fromhex(values[7]))[0],
            struct.unpack('<f', bytes.fromhex(values[8]))[0],
        )
        quaternion.orient_x = orient_x

        ## Position
        position: Point_t = Point_t(
            struct.unpack('<i', bytes.fromhex(values[9]))[0],
            struct.unpack('<f', bytes.fromhex(values[10]))[0],
            struct.unpack('<f', bytes.fromhex(values[11]))[0],
            struct.unpack('<f', bytes.fromhex(values[12]))[0],
        )

        ## Velocity
        velocity: Velocity_t = Velocity_t(
            struct.unpack('<i', bytes.fromhex(values[13]))[0],
            struct.unpack('<f', bytes.fromhex(values[14]))[0],
            struct.unpack('<f', bytes.fromhex(values[15]))[0],
            struct.unpack('<f', bytes.fromhex(values[16]))[0],
        )

        ## Acceleration
        acc: Acc_t = Acc_t(
            struct.unpack('<i', bytes.fromhex(values[17]))[0],
            struct.unpack('<f', bytes.fromhex(values[18]))[0],
            struct.unpack('<f', bytes.fromhex(values[19]))[0],
            struct.unpack('<f', bytes.fromhex(values[20]))[0],
        )

        ## State
        state: State_t = State_t(
            attitude,
            quaternion,
            position,
            velocity,
            acc
        )

        DEBUG_PRINT_CONTROL(thrust, state)

        return (thrust, state)



    def _handle_control_packet(self, payloadLength: int) -> Optional[Tuple[float, State_t]]:
        """
        Handles the reception and processing of a control packet.

        This function reads a control packet from the serial connection, verifies its
        length and CRC, and processes the payload. If the CRC is correct, it sends the
        packet back; otherwise, it sends an error packet. If no data is received, it
        sends a flag and resets the communication state to SYNC.

        Args:
            payloadLength (int): The expected length of the payload in bytes.

        Returns:
            Optional[Tuple[float, State_t]]: The decomposed payload data if the CRC is correct,
            otherwise None.

        The function performs the following steps:
        1. Reads the maximum payload length from the serial connection.
        2. Extracts the useful payload data based on the provided payload length.
        3. Checks if the read data length matches the expected payload length.
        4. Decomposes the payload data and reads the CRC byte.
        5. Verifies the CRC and sends the packet back if correct, otherwise sends an error packet
           and the communication state will be reseted to SYNC.
        6. If no data is received, sends a flag and resets the communication state to SYNC.

        Prints the payload length, payload size, and status messages for debugging purposes.
        """

        print(f'Useful payload length in bytes: {payloadLength}')

        data_all = self.ser.read(MAX_PAYLOAD_LENGTH)
        print(f'Payload size in bytes: {len(data_all)}')
        data = data_all[0:payloadLength]

        
        ## Check if the read data has the same length as expected
        ## If its not then a timeout
        if len(data) == payloadLength:
            
            self._control_payload_decomp_static(data=data, payload_length=payloadLength)
            crc = int(self.ser.read(1).hex(), 16)

            #crc = data_all[payloadLength]

    
            ## If crc is correct return data
            ## Else return wrong crc flag (0xFE)
            if self._handle_crc(crc_value = crc, data = data):
                self._send_packet(data, 0x01)
            else:
                self._send_error()
        
        else:
            ## If no data is received send back a flag then
            ## go back to sync state
            packet = b''
            packet += struct.pack('B', 0xFE)
            self.ser.write(packet)
            self.comm_state = CommState.SYNC
            print('No data received.')



    def _trajectory_payload_decomp_static(self, data: bytes, payload_length: int) -> Optional[List[float]]:
        values = [data[i : i+4].hex() for i in range(0, payload_length, 4)]
        dummy_list: List[float] = []
        dummy_list.append(struct.unpack('<f', bytes.fromhex(values[0]))[0])
        dummy_list.append(struct.unpack('<f', bytes.fromhex(values[1]))[0])
        dummy_list.append(struct.unpack('<f', bytes.fromhex(values[2]))[0])
        
        DEBUG_PRINT_TRAJECTORY(dummy_list)

        return dummy_list




    def _handle_trajectory_packet(self, payloadLength: int) -> Optional[List[float]]:
        print(f'Useful payload length in bytes: {payloadLength}')

        data_all = self.ser.read(MAX_PAYLOAD_LENGTH)
        print(f'Payload size in bytes: {len(data_all)}')
        data = data_all[0:payloadLength]

        
        ## Check if the read data has the same length as expected
        ## If its not then a timeout
        if len(data) == payloadLength:
            
            self._trajectory_payload_decomp_static(data=data, payload_length=payloadLength)
            crc = int(self.ser.read(1).hex(), 16)

            #crc = data_all[payloadLength]


            ## If crc is correct return data
            ## Else return wrong crc flag (0xFE)
            if self._handle_crc(crc_value = crc, data = data):
                self._send_packet(data, 0x02)
            else:
                self._send_error()
        
        else:
            ## If no data is received send back a flag then
            ## go back to sync state
            packet = b''
            packet += struct.pack('B', 0xFE)
            self.ser.write(packet)
            self.comm_state = CommState.SYNC
            print('No data received.')
        



def DEBUG_PRINT_CONTROL(thrust: float = None, state: State_t = None) -> None:
    if thrust is None or state is None:
        return
    
    print(f'Thrust: {thrust}')
    print(f'Attitude: {state.attitude.to_dataclass()}')
    print(f'Attitude quaternion: {state.attitudeQuaternion.orient_x.to_dataclass()}')
    print(f'Position: {state.position.to_dataclass()}')
    print(f'Velocity: {state.velocity.to_dataclass()}')
    print(f'Acceleration: {state.acc.to_dataclass()}')


def DEBUG_PRINT_TRAJECTORY(list: List[float]) -> None:
    for i in range(len(list)):
        print(f'Dummy float {i + 1}: {list[i]}')



if __name__ == '__main__':
    
    comm = UARTCommunicationV2(SERIAL_URI, BAUD_RATE)
    comm.communicate()
    comm.close_commmunication()
    



