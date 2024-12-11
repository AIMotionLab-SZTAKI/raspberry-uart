from uart.uart_serial import UARTCommunication
from utils.config import SERIAL_URI, BAUD_RATE
from utils.util import ControlFlag
from crazyradio import RadioReciever
from multiprocessing import shared_memory, Lock, Process
import numpy as np
import traceback
import time


if __name__ == "__main__":
    # SERIAL_URI and BAUD_RATE can be modified in utils/config.py
    
    shm = shared_memory.SharedMemory(create=True, size=5 * np.dtype(np.float64).itemsize)
    shared_array = np.ndarray((5,), dtype=np.float64, buffer=shm.buf)
    shared_array.fill(0)
    
    lock = Lock()
    
    uart = UARTCommunication(SERIAL_URI, BAUD_RATE)  # two-way UART communication with Crazyflie
    uart_process = Process(target=uart.communicate, args=(shm.name, lock), daemon=True)

    receiver = RadioReciever(devid=0)  # receiving radio messages from PC
    
    try:
        uart_process.start()
        while True:
            rec = receiver.receive()
            if rec is not None:
                # print(np.array(rec))
                np.copyto(shared_array, np.array(rec))
                time.sleep(0.0001)

    except Exception as exc:
        print(f"Exception: {exc!r}. TRACEBACK:\n")
        print(traceback.format_exc())
        receiver.radio.close()
        uart.close_commmunication()
