from openteach.ros_links.bimanual import DexArmControl
from .robot import RobotWrapper
from openteach.utils.network import ZMQKeypointSubscriber
import numpy as np
import time


class RM65R(RobotWrapper):
    def __init__(self, ip, record_type=None):
        self._controller = DexArmControl(ip=ip, record_type=record_type)
        self._data_frequency = 90
        self.name = "RM65_R"

    @property
    def data_frequency(self):
        return self._data_frequency

    @property
    def recorder_functions(self):
        return {
            
        }
