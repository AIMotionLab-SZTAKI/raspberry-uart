# UART communication module

This module enables communication between the Crazyflie and a Raspberry Pi via UART.


## PDU

The default PDU structure for the UART communication is shown in the table below.

| `START` | `SERVICE TYPE` | `PAYLOAD LENGTH` | `PAYLOAD` | `CRC` |
|:-------:|:--------------:|:----------------:|:---------:|:-----:|
| 1 Byte  |    1 Byte      |     1 Byte       |  Max 200 Byte| 1 Byte|


The payload size is fixed at 200 bytes due to the Crazyflie firmware implementation, though this can be modified. Both sides transmit only the necessary amount of payload to further speed up communication.



## Raspberry Pi modules

The UART communication modules on the Raspberry Pi were written in Python. The main components are listed below.

##### ```communication_packets.py```

It defines a communication packet using Python ctypes. A factory method is used to instantiate a packet, allowing for the creation of a packet with a variable payload size.


##### ```stabilizer_types.py```

It defines the Python c_types implementations of the data structures defined in the Crazyflie firmware stabilizer_type.h file.

***Note that at this point not every structure is implemented.***

##### ```congig.py```

This module defines constants for configuring the serial interface.

*  **Serial Configuration Constans:**
    * `SERIAL_URI`: Specifies the device file for the serial interface (e.g., '/dev/ttyAMA0').

    * `BAUD_RATE`: Sets the communication speed (921600 bits per second).

    * `READ_TIMEOUT` and `WRITE_TIMEOUT`: Define the time limits for read and write operations. `READ_TIMEOUT` is set to 0.1 seconds, while `WRITE_TIMEOUT` is unspecified (None).

* **Communication Packet Constants:**
    * `MAX_UART_BUFFER_SIZE`: The maximum size of the UART buffer, set to 200 bytes.

    * `PACKET_HEADER_LENGTH`: Length of the packet header (3 bytes).

    * `PACKET_CRC_LENGTH`: Length of the CRC for error checking (1 byte).

    * `PACKET_META_LENGTH`: Total length of the header and CRC combined.

    * `MAX_PAYLOAD_LENGTH`: The maximum length of the payload data, calculated by subtracting the meta length from the maximum buffer size.

##### ```util.py```

This module define enums that are used during communication to signal the states of the communication.

* **CommState:**
    * `SYNC`
    * `COMM`

* **ServiceType:**
    * `CONTROL`
    * `TRAJECTORY`

* **ControlFlag:**
    * `SYNC_OK`
    * `START`

* **ErrorFlag:**
    * `BAD_CRC`
    * `NO_DATA_RECEIVED`


***Control flags and error flags have separate enums, even though the same method is used to send them. This distinction is necessary because when an error flag is sent, the communication should reset to the SYNC state.***

***If new service types, error flags, or control flags are developed in the future, these enums should be updated accordingly.***

##### ```packet_handler_interface.py```

This module defines an interface for packet handlers. When a new service is introduced, a new packet handler class should be created. This new class must inherit from the interface class and implement the required methods.

Every handler should implement the following two methods.

* `packet_decomposition`: This method decomposes the received packet's payload.

* `packet_composition`: This method creates the payload for the packet that will be send.

***These methods should be implemented according to the service type.***



##### ```uart_serial.py```

This module is responsible for establishing the serial interface, managing synchronization, and facilitating communication.

* Synchronization: It waits for a synchronization byte (SYNC_OK) from the Crazyflie. Upon receiving it, the module changes the communication state from SYNC to COMM and sends back an acknowledgment byte (SYNC_OK) to inform the Crazyflie.

* Communication: Once synchronized, this module manages communication. It reads the header data and processes the payload according to the header metadata.


* Packet handling: After receiving a packet with a correct header the `_handle_packet()` method is called. This process the packet according to the payload. If no error occurs `_send_packet()` method is called, it creates packet and sends it back to the Crazyflie.

* Error handling: If an error occurs during communication (e.g, Bad crc) an ErrorFlag is send using the `_send_flag()` method. This same method is used to send back ControlFlags.


## C modules

To communicate via UART with the Raspberry Pi, a new task is created, along with an interface for the module.

##### `communication.c`

This file, communication.c, is part of the Crazyflie firmware and is responsible for managing UART communication. It includes various headers for system, logging, and communication functionalities. The file defines constants for baud rate, queue length, and packet sizes, and declares static variables for initialization status, communication state, and synchronization bytes.


* `communicationInit()`: Initializes the UART communication with a predefined baud rate, creates mutexes and queues, and starts the communication task. Ensures initialization is performed only once.


* `communicationTask()`: Manages the communication loop, handling synchronization, sending and receiving packets, and processing received data based on service type. It waits for the system to start, then enters a loop to handle communication until shutdown is requested.

* `calcCrc()`: Calculates the CRC for a given UART packet by performing an XOR operation on each byte of the packet's payload.

* `createControlPacket()`: Creates a control packet.

* `createTrajectoryPacket()`: Creates a trajectory packet.

* `sendDataUART()`: Sends data over UART based on the specified format, creating appropriate packets and sending them to the transmission queue.

* `receiveDataUART()`: Receives data from the UART, checking the communication state and processing received packets.

* `communicationTest()`: Checks if the communication module has been initialized.

##### `Static Variables`:

* `isInit`: Indicates if the communication module has been initialized.

* `shutdownTransport`: Flag to indicate if the communication should be shut down.

* `SYNC_BYTE`, `START_BYTE`, `DEFAULT_BYTE`: Constants for synchronization and packet start.

* `pckDataMutex`, `pckDataMutexBuffer`: Mutex for packet data protection.

* `txQueue`, `rxQueue`: Queues for transmitting and receiving packets.

* `commState`: Current state of communication (e.g., SYNC, CONNECTED).

* `comm_timeout`: Timeout value for communication operations.


##### `communication.h`

This header file defines constants, types, and function prototypes for UART communication. It includes standard and custom headers, defines buffer sizes and packet structure, and specifies packet types. The CommState enum represents communication states, while the uart_packet struct defines the structure of a UART packet. Function prototypes for initializing communication, sending and receiving data, testing communication, and handling control packets are also provided.

## Usage

This section explains how to create a new service type and use the UART communication interface.

### `Firmware side`

* In `communication.h` define the new packet tpye. (e.g. control packet)
```c
#define CONTROL_PACKET 0x01
```

* In `communication.c` create a packetCreate function, which creates a UART packet. (e.g. createControlPacket() )
```c
void createControlPacket(uart_packet* packet, va_list* args) {
    // get variables from va_list

    ASSERT(thrust != NULL && state != NULL);

    // calculate paylaod length
    ASSERT(payloadLength <= MAX_PAYLOAD_LENGTH);

    // assign variables to the UART packet
    packet->start = START_BYTE;  
    packet->serviceType = CONTROL_PACKET;
    ...

    unsigned long ptr = 0;

    // put the data to the packet's payload field
    memcpy(&packet->payload[ptr], thrust, sizeof(*thrust));
    ptr += sizeof(*thrust);
    ...
    
    // Put the crc byte at the end of the payload.
    // This is necessary to ensure that only the required
    // amount of data is sent through the UART.
    packet->payload[packet->payloadLength] = calcCrc(packet);
    
}
```

* In `communication.c` create a packet handler function which manages the receives packet based on its service type. (e.g. void handle_control_packet() )
```c
void handle_control_packet(uart_packet *packet, float* thrustDesired, ...) {
    unsigned long ptr = 0;

    // memcpy the payload from the UART packet to the variables
    memcpy(thrustDesired, &packet->payload[ptr], sizeof(*thrustDesired));
    ptr += sizeof(*thrustDesired);
    ptr += 4; // timestamp sent as an integer
    ...
}
```

* Define the handler function in `communication.h`.
```c
void handle_control_packet(uart_packet *packet, float* thrustDesired, float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);
```

* In the `sendDataUART` function (`communication.c`) create a new case code block and call the packet create function. (e.g. createControlPacket() ).
Choose a format string/character which'll be used when creating a packet according to the service type (e.g. control-> 'C', trajectory -> 'T', etc.)
```c
switch (*format++) {
      case 'C': { 
        // Control (thrust, state) 
        createControlPacket(&packet, &args);
        break;
      }
      ...
```

* In the `controller_pid.c` call the `sendDataUART` and the handler fucntion.
```c
  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    // call the sendDataUART function whith the correct format string and parameters
    sendDataUART("C", &actuatorThrust, state);
    

    uart_packet receiverPacket;
    if (receiveDataUART(&receiverPacket)) {
      if (receiverPacket.serviceType == CONTROL_PACKET) {
        handle_control_packet(&receiverPacket, &thrust_ext, &rateDesired_ext.roll, &rateDesired_ext.pitch, &rateDesired_ext.yaw);
      }
      else if (...) {
        // another packet type
      }
    }
    
  }
```

### `Raspberry's side`

* On the Raspberry's side create, in the `util/debug.py` add a new enum variable to the `ServiceType` enum.
```python
class ServiceTpye(Enum):
    CONTROL = 1
    TRAJECTORY = 2
    ...
```

* In the `packet_handlers` module create a new python file and class and redefine `PacketHandlerInterface` interface. Define the methods according to the service type. (e.g.  ControlPacketHandler)
```python
class ControlPacketHandler(PacketHandlerInterface):
    
    @staticmethod
    def packet_composition(thrust: float, attitude_rate: Attitude_t) -> bytes:
        # #Assemble the payload data for the packet that will be sent
        data = struct.pack('<fifff', thrust, attitude_rate.timestamp, attitude_rate.roll, attitude_rate.pitch, attitude_rate.yaw)
        return data
    

    @staticmethod
    def packet_decomposition(data: bytes, payload_length: int) -> Optional[Tuple[float, State_t]]:
        ## decompose the payload of the received packet
        if payload_length % 4 != 0:
            return
        
        values = [data[i : i+4] for i in range(0, payload_length, 4)]
        #print(values)
        ## Thrust
        thrust: float = struct.unpack('<f',values[0])[0] # thrust

        ...
```

* At the `uart/uart_serial.py` in the `_handle_packet` method call the above defined interface's methods.
```python
def _handle_packet(self, payloadLength: int, service_type: int) -> Optional[Tuple[float, State_t]]:

        ...
        
        if len(data) == payloadLength:
            
            if service_type == ServiceType.CONTROL.value:    
                print('Service type: CONTROL')
                ## call the packet decomposition method in the correct if-elif branch
                _, state = ControlPacketHandler.packet_decomposition(data, payloadLength)
                ...

            elif service_type == ServiceType.TRAJECTORY.value:
                print('Service type: TRAJECTORY')
                TrajectoryPacketHandler.packet_decomposition(data, payloadLength)

            else:
                print("Service type unknown.")
                self.reset_communication()

            ...
            
            ## If crc is correct return data
            ## Else return wrong crc flag (0xFE)
            if self._handle_crc(crc_value = crc, data = data):
                if service_type == ServiceType.CONTROL.value: 
                    ## call the packet composition method in the correct if-elif branch
                    data = ControlPacketHandler.packet_composition(thrust, attitude_rate)

                elif service_type == ServiceType.TRAJECTORY.value:
                    data = TrajectoryPacketHandler.packet_composition(88.88, 99.99)
                
                ## send packet method's creates a UART packet with the data variables as payload and sends it
                self._send_packet(data, service_type)

                ...
```
