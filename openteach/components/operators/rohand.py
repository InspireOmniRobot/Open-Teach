from copy import deepcopy as copy

import numpy as np
from numba import njit
from scipy.spatial.transform import Rotation, Slerp
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

from openteach.constants import *
from openteach.robot.rohand import Rohand
from openteach.utils.files import *
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from openteach.utils.timer import FrequencyTimer
from openteach.utils.vectorops import coord_in_bound

from .calibrators.allegro import OculusThumbBoundCalibrator
from .operator import Operator

np.set_printoptions(precision=2, suppress=True)


# @njit(fastmath=True, cache=True)
def calculate_angle(points):
    # 5 point sets, each set has 3 points, xyz coordinates
    vector_1 = points[:, 1, :] - points[:, 0, :]
    norm_1 = np.linalg.norm(vector_1, axis=1)

    vector_2 = points[:, 2, :] - points[:, 1, :]
    norm_2 = np.linalg.norm(vector_2, axis=1)

    dot_product = np.sum(vector_1 * vector_2, axis=1)

    angle = np.arccos(dot_product / (norm_1 * norm_2))
    return angle


# Filter for removing noise in the teleoperation
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (
            1 - self.comp_ratio
        )
        ori_interp = Slerp(
            [0, 1],
            Rotation.from_rotvec(np.stack([self.ori_state, next_state[3:6]], axis=0)),
        )
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
        return np.concatenate([self.pos_state, self.ori_state])


# Template arm operator class
class RohandOnlyOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        use_filter=False,
        finger_configs=None,
        *args,
        **kwargs,
    ):
        print(finger_configs)
        print(args)
        print(kwargs)
        self.notify_component_start("rohand operator")
        self._host, self._port = host, transformed_keypoints_port
        # Subscriber for the transformed hand keypoints
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=self._host, port=self._port, topic="transformed_hand_coords"
        )
        # Initializing the robot controller
        self._robot = Rohand()
        self.finger_configs = finger_configs
        self._timer = FrequencyTimer(VR_FREQ)

        self.max_ratio = 65535.0 / np.array([10, 1.6, 2.0, 2.0, 0.5])
        self.deviation = np.array([0.32, 0.31, 0.12, 0.13, 0.44], dtype=np.float32)
        # self.max_ratio = 1

        self._joints = [
            OCULUS_JOINTS["thumb"][0],
            OCULUS_JOINTS["thumb"][-1],
            OCULUS_JOINTS["index"][0],
            OCULUS_JOINTS["index"][-1],
            OCULUS_JOINTS["middle"][0],
            OCULUS_JOINTS["middle"][-1],
            OCULUS_JOINTS["ring"][0],
            OCULUS_JOINTS["ring"][-1],
            OCULUS_JOINTS["pinky"][0],
            OCULUS_JOINTS["pinky"][-1],
        ]
        # print(self._joints)

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    # This function differentiates between the real robot and simulation
    def return_real(self):
        return True

    # Get the transformed finger coordinates
    def _get_finger_coords(self):
        raw_keypoints = self.transformed_hand_keypoint_subscriber.recv_keypoints()
        extracted_keypoints = raw_keypoints[self._joints]

        points_pairs = np.array(
            [
                (raw_keypoints[0], extracted_keypoints[0], extracted_keypoints[1]),
                (raw_keypoints[0], extracted_keypoints[2], extracted_keypoints[3]),
                (raw_keypoints[0], extracted_keypoints[4], extracted_keypoints[5]),
                (raw_keypoints[0], extracted_keypoints[6], extracted_keypoints[7]),
                (raw_keypoints[0], extracted_keypoints[8], extracted_keypoints[9]),
            ]
        )  # 5*3*3

        angles = calculate_angle(points_pairs)
        angles = angles - self.deviation
        angles = np.clip(angles, 0, 3.14)
        angles = self.max_ratio * angles
        return np.clip(angles, 0, 65535)

    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        hand_keypoints = self._get_finger_coords().astype(int)
        final_result = list(hand_keypoints) + [0]
        print(final_result)
        # return
        self.robot.move(final_result)
