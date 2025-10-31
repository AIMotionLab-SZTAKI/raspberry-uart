from uart.uart_serial import UARTCommunication
from utils.config import SERIAL_URI, BAUD_RATE
from utils.util import ControlFlag
from packet_handlers.forward_packet_handler import ForwardPacketHandler
from crazyradio import RadioReciever, Crazyradio
from multiprocessing import shared_memory, Lock, Process
import numpy as np
import traceback
import time
import os

if __name__ == "__main__":
    # SERIAL_URI and BAUD_RATE can be modified in utils/config.py
    
    shm = shared_memory.SharedMemory(create=True, size=ForwardPacketHandler.packet_size * np.dtype(np.float64).itemsize)
    shared_array = np.ndarray((ForwardPacketHandler.packet_size,), dtype=np.float64, buffer=shm.buf)
    shared_array.fill(0)

    # prepare pwm log file
    logfile = os.path.join(os.path.dirname(__file__), "pwm_log.jsonl")
    # clear file at startup
    open(logfile, "w").close()
    
    lock = Lock()
    
    uart = UARTCommunication(SERIAL_URI, BAUD_RATE)  # two-way UART communication with Crazyflie
    uart_process = Process(target=uart.communicate, args=(shm.name, lock), daemon=True)

    receiver = RadioReciever(devid=0, channel=90, data_rate=Crazyradio.DR_2MPS)  # receiving radio messages from PC
    
    try:
        uart_process.start()
        while True:
            rec = receiver.receive(size=ForwardPacketHandler.packet_size)
            if rec is not None:
                # print(np.array(rec))
                np.copyto(shared_array, np.array(rec))
                time.sleep(0.0001)

    except Exception as exc:
        print(f"Exception: {exc!r}. TRACEBACK:\n")
        print(traceback.format_exc())
        receiver.close()
        uart.close_commmunication()
        shm.unlink()
