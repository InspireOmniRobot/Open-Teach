import numpy as np
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
import openteach.ros_links.roh_registers_v1 as roh

COM_PORT = "/dev/ttyUSB0"
NODE_ID = 2


ROHAND_ORIGINAL_HOME_VALUES = [0, 0, 0, 0, 0, 0]

# if __name__ == "__main__":
#     # 握拳
#     resp = client.write_registers(ROH_FINGER_POS_TARGET0, [50000, 45000, 65535, 65535, 65535, 12000], NODE_ID)


class DexArmControl:
    def __init__(self):
        # if pub_port is set to None it will mean that
        # this will only be used for listening to franka and not commanding
        self.client = ModbusSerialClient(COM_PORT, framer=FramerType.RTU, baudrate=115200)
        assert self.client.connect()
        self.client.write_registers(
            roh.ROH_FINGER_POS_TARGET0, values=ROHAND_ORIGINAL_HOME_VALUES, slave=NODE_ID
        )

    def get_hand_position(self):
        resp = self.client.read_holding_registers(roh.ROH_FINGER_POS0, count=6, slave=NODE_ID)
        return np.array(resp.registers, dtype=np.float32)

    def get_hand_velocity(self):
        resp = self.client.read_holding_registers(roh.ROH_FINGER_SPEED0, count=6, slave=NODE_ID)
        return np.array(resp.registers, dtype=np.float32)

    # Movement functions
    def move_hand(self, rohand_values):
        self.client.write_registers(roh.ROH_FINGER_POS_TARGET0, count=rohand_values, slave=NODE_ID)

    def reset_hand(self):
        self.move_hand(ROHAND_ORIGINAL_HOME_VALUES)
