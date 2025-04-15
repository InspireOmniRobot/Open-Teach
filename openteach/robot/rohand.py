from openteach.ros_links.rohand import DexArmControl

from .robot import RobotWrapper


class Rohand(RobotWrapper):
    def __init__(self):
        self._controller = DexArmControl()
        self._data_frequency = 60

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state,
            "cartesian_states": self.get_cartesian_state,
        }

    @property
    def name(self):
        return "rohand"

    @property
    def data_frequency(self):
        return self._data_frequency

    def get_hand_position(self):
        return self._controller.get_hand_position()

    def get_hand_velocity(self):
        return self._controller.get_hand_velocity()

    # Movement functions
    def move_hand(self, rohand_angles):
        return self._controller.move_hand(rohand_angles)

    def reset(self):
        return self._controller.reset_hand()

    def home(self):
        return self.reset()

    def move_coords(self, fingertip_coords):
        raise Exception("Not implemented")
        self._controller.move_hand(fingertip_coords)

    def move(self, angles):
        self._controller.move_hand(angles)

    def get_joint_state(self):
        return self._controller.get_hand_position()

    def get_joint_position(self):
        return self._controller.get_hand_position()
