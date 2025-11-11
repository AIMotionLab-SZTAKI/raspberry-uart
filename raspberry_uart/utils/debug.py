from typing import List
from raspberry_uart.datatypes.stabilizer_types import *

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