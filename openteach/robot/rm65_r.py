import time

import numpy as np

from openteach.ros_links.rm65_bi import DexArmControl
from openteach.utils.network import ZMQKeypointSubscriber

from .robot import RobotWrapper


class RM65R(RobotWrapper):
    def __init__(self, robot_ip, robot_port):
        super().__init__()
        self._controller = DexArmControl(ip=robot_ip, port=robot_port, arm_type="right")

    @property
    def data_frequency(self):
        return 30

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state_from_socket,
            "cartesian_states": self.get_cartesian_state_from_socket,
            "gripper_states": self.get_gripper_state_from_socket,
            "actual_cartesian_states": self.get_robot_actual_cartesian_position,
            "actual_joint_states": self.get_robot_actual_joint_position,
            "actual_gripper_states": self.get_gripper_state,
            "commanded_cartesian_state": self.get_cartesian_commanded_position,
        }

    @property
    def name(self):
        return "RM65_R"

    # State information functions
    def get_joint_state(self):
        return self._controller.get_arm_joint_state()

    def get_joint_velocity(self):
        raise NotImplementedError("get_joint_velocity() is not implemented for RM65")

    def get_joint_torque(self):
        raise NotImplementedError("get_joint_torque() is not implemented for RM65")

    def get_cartesian_state(self):
        return self._controller.get_cartesian_state()

    def get_joint_position(self):
        return self._controller.get_arm_joint_position()

    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_position()

    def reset(self):
        return self._controller.reset_arm()

    def get_pose(self):
        return self._controller.get_arm_pose_affine()

    # Movement functions
    def home(self):
        return self._controller.home_arm()

    def move(self, input_angles):
        self._controller.move_arm_joint(input_angles)

    def move_coords(self, cartesian_coords):
        self._controller.move_arm_cartesian(cartesian_coords)

    def move_coords_quad(self, cartesian_coords):
        self._controller.move_arm_cartesian_quad(cartesian_coords)

    def set_gripper_state(self, gripper_state):
        self._controller.set_gripper_position(gripper_state)

    def get_gripper_state_from_socket(self):
        self._gripper_state_subscriber = ZMQKeypointSubscriber(
            host="10.19.216.156", port=8108, topic="gripper_right"
        )
        gripper_state = self._gripper_state_subscriber.recv_keypoints()
        gripper_state_dict = dict(
            gripper_position=np.array(gripper_state, dtype=np.float32),
            timestamp=time.time(),
        )
        return gripper_state_dict

    def get_cartesian_state_from_socket(self):
        self.cartesian_state_subscriber = ZMQKeypointSubscriber(
            host="10.19.216.156", port=8118, topic="cartesian"
        )
        cartesian_state = self.cartesian_state_subscriber.recv_keypoints()
        cartesian_state_dict = dict(
            cartesian_position=np.array(cartesian_state, dtype=np.float32),
            timestamp=time.time(),
        )
        return cartesian_state_dict

    def get_joint_state_from_socket(self):
        self._joint_state_subscriber = ZMQKeypointSubscriber(
            host="10.19.216.156", port=8119, topic="joint"
        )
        joint_state = self._joint_state_subscriber.recv_keypoints()
        joint_state_dict = dict(
            joint_position=np.array(joint_state, dtype=np.float32),
            timestamp=time.time(),
        )
        return joint_state_dict

    def get_cartesian_commanded_position(self):
        self.cartesian_state_subscriber = ZMQKeypointSubscriber(
            host="10.19.216.156", port=8120, topic="cartesian"
        )
        cartesian_state = self.cartesian_state_subscriber.recv_keypoints()
        cartesian_state_dict = dict(
            commanded_cartesian_position=np.array(cartesian_state, dtype=np.float32),
            timestamp=time.time(),
        )
        return cartesian_state_dict

    def get_robot_actual_cartesian_position(self):
        cartesian_state = self.get_cartesian_position()
        cartesian_dict = dict(
            cartesian_position=np.array(cartesian_state, dtype=np.float32),
            timestamp=time.time(),
        )

        return cartesian_dict

    def get_robot_actual_joint_position(self):
        joint_state_dict = self._controller.get_arm_joint_state()
        return joint_state_dict

    def get_gripper_state(self):
        gripper_state_dict = self._controller.get_gripper_state()
        return gripper_state_dict
