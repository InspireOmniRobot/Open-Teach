import time
from enum import Enum
import numpy as np
from Robotic_Arm.rm_robot_interface import (
    Algo,
    RoboticArm,
    rm_thread_mode_e,
    rm_inverse_kinematics_params_t,
    rm_robot_arm_model_e,
    rm_force_type_e,
    rm_frame_t,
)
from scipy.spatial.transform import Rotation as R
from openteach.constants import BIMANUAL_RM65
import socket
import json
import logging

logging.basicConfig(level=logging.INFO, filename="rm65_bi.log")
logger = logging.getLogger("RightArm")


class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1


# Wrapper for Realman
class Robot:
    def __init__(self, ip, port, arm_type):
        self.arm_type = arm_type
        # self.ratio = 1000
        # self.arm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.arm_socket.connect((ip, port))
        # 初始化算法的机械臂及末端型号
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        _handle = self.arm.rm_create_robot_arm(ip, port)
        self.last_time = time.perf_counter()

        if arm_type == "left":
            self.constants = BIMANUAL_RM65.L
        elif arm_type == "right":
            self.constants = BIMANUAL_RM65.R
        else:
            raise ValueError("Invalid arm type")

        # _software_info = self.arm.rm_get_arm_software_info()

        # if _software_info[0] == 0:
        #     print("\n================== Arm Software Information ==================")
        #     print("Robot arm id", _handle.id)
        #     print(
        #         "Algorithm Library Version: ",
        #         _software_info[1]["algorithm_info"]["version"],
        #     )
        #     print(
        #         "Control Layer Software Version: ",
        #         _software_info[1]["ctrl_info"]["version"],
        #     )
        #     print("Dynamics Version: ", _software_info[1]["dynamic_info"]["model_version"])
        #     print(
        #         "Planning Layer Software Version: ",
        #         _software_info[1]["plan_info"]["version"],
        #     )
        #     print("==============================================================\n")
        # else:
        #     raise Exception("Failed to get Realman RM65 arm software information, Error code: ", _software_info[0])

        # 设置机械臂为仿真模式
        self.arm.rm_set_arm_run_mode(1)
        self.arm.rm_set_self_collision_enable(True)

        # algo_handle must be initialized after arm is initialized
        self.algo_handle = Algo(
            rm_robot_arm_model_e.RM_MODEL_RM_65_E, rm_force_type_e.RM_MODEL_RM_B_E
        )
        self.algo_handle.rm_algo_set_redundant_parameter_traversal_mode(False)
        _, tool_frame = self.arm.rm_get_current_tool_frame()
        self.algo_handle.rm_algo_set_toolframe(rm_frame_t(pose=tool_frame["pose"]))

    # def scale_angle_up(self, angles):
    #     return [int(i * self.ratio) for i in angles]

    # def scale_angle_down(self, angles):
    #     return [i / self.ratio for i in angles]

    # def send_command(self, command):
    #     data_json = json.dumps(command).encode("utf-8")
    #     self.arm_socket.sendall(data_json)
    #     response = self.arm_socket.recv(1024).decode("utf-8")  # 接收响应（可选）
    #     # print("- - - - - - - - - - ")
    #     # print(response)
    #     # print(data_json)
    #     # print("- - - - - - - - - - ")
    #     logger.info(f"Send command: {command}")
    #     logger.info(f"Response: {response}")
    #     return json.loads(response)

    def reset(self):
        # home_js = self.scale_angle_up(self.constants.HOME_JS)
        # command = {"command": "movej", "joint": home_js, "v": 5, "r": 0, "trajectory_connect": 0}
        # self.send_command(command)
        self.arm.rm_movej(self.constants.HOME_JS, v=5, r=0, connect=0, block=1)

    def set_gripper_position(self, position):
        # self.rm_set_gripper_position(position)
        raise NotImplementedError(
            "set_gripper_position() is not yet implemented for Realman RM65"
        )

    def get_gripper_position(self):
        # self.rm_get_gripper_position()
        raise NotImplementedError(
            "get_gripper_position() is not yet implemented for Realman RM65"
        )

    def get_current_joint_state(self):
        code, joint_state = self.arm.rm_get_joint_degree()
        assert code == 0, "Error getting joint_degree"
        return joint_state[:6]
        # command = {"command": "get_joint_degree"}
        # while True:
        #     resp = self.send_command(command)
        #     joint = resp.get("joint", False)
        #     if joint:
        #         joint = self.scale_angle_down(joint)
        #         return joint
        #     else:
        #         logger.info("Failed to get joint state")

    def get_current_pose_state(self):
        current_joint_state = self.get_current_joint_state()
        current_pose = self.algo_handle.rm_algo_forward_kinematics(
            current_joint_state, flag=1
        )
        return np.array(current_pose, dtype=np.float32)

    def move_arm_cartesian(self, cartesian_pos):
        cartesian_pos = np.round(cartesian_pos, 2)
        """Move a joint in METERS !!!"""
        current_joint = self.get_current_joint_state()
        param = rm_inverse_kinematics_params_t(current_joint, cartesian_pos, flag=1)
        code, joint = self.algo_handle.rm_algo_inverse_kinematics(param)
        # logger.info(f"inverse kinematics: {current_joint}, {cartesian_pos}, {joint}")

        if code != 0:
            print(f"Failed to calculate inverse kinematics: {code}, {cartesian_pos}")
        else:
            print(
                "Final joint: ", np.round(cartesian_pos, 2), np.array(joint, dtype=int)
            )
            # resp = self.send_command({"command": "movej_canfd", "joint": self.scale_angle_up(joint), "follow": False})
            # assert resp["arm_err"] == 0, "Failed to move arm cartesian"
            self.arm.rm_movej_canfd(joint, follow=False)
            time_now = time.perf_counter()
            frequency = 1 / (time_now - self.last_time)
            logger.info(f"Frequency: {frequency} fps")
            self.last_time = time_now


class DexArmControl:
    def __init__(self, ip, port, arm_type):
        self.robot = Robot(ip, port, arm_type)

    def move_arm_cartesian(self, cartesian_pos):
        """Move a joint in METERS and AA !!!"""
        return self.robot.move_arm_cartesian(cartesian_pos)

    def move_arm_cartesian_quad(self, cartesian_pos):
        """Move a joint in METERS and QUAD !!!"""
        return self.robot.move_arm_cartesian(cartesian_pos, quad=True)

    def move_arm_joint(self, joint_pos):
        """Move a joint in DEGREE !!!"""
        return self.robot.move_arm_joint(joint_pos)

    # State information functions

    # def get_arm_pose_affine(self):
    #     home_pose = self.get_arm_position()
    #     home_affine = self.robot_pose_aa_to_affine(home_pose)
    #     return home_affine

    def get_arm_cartesian_position(self):
        # return np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        return self.robot.get_current_pose_state()

    def get_arm_joint_position(self):
        return self.robot.get_current_joint_state()

    def get_gripper_state(self):
        raise NotImplementedError(
            "get_gripper_state() is not yet implemented for Realman RM65"
        )
        gripper_position = self.robot.get_gripper_position()
        gripper_pose = dict(
            position=np.array(gripper_position[1], dtype=np.float32).flatten(),
            timestamp=time.time(),
        )
        return gripper_pose

    def get_arm_joint_state(self):
        joint_positions = self.robot.get_current_joint_state()
        return {
            "position": joint_positions,
            "timestamp": time.time(),
        }

    def get_cartesian_state(self):
        current_pos = self.robot.get_current_pose_state()
        return {
            "position": current_pos[0:3],
            "orientation": current_pos[3:],
            "timestamp": time.time(),
        }

    def home_arm(self):
        self.robot.reset()

    def reset_arm(self):
        self.home_arm()

    def home_robot(self):
        self.home_arm()  # For now we're using cartesian values

    def move_robot(self, arm_angles):
        # self.robot.move_arm_joint(arm_angles)
        raise NotImplementedError("move_robot() is not implemented for Realman RM65")

    def set_gripper_state(self, position):
        # self.robot.set_gripper_position(position)
        pass
        # raise NotImplementedError("set_gripper_status() is not implemented for Realman RM65")

    def robot_pose_aa_to_affine(self, pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
        Returns:
            np.ndarray: 4x4 affine matrix
        """
        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = pose_aa[:3]

        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = translation

        return result


# if quad:
#     pos = cartesian_pos[:3] + self.arm.rm_algo_euler2quaternion(cartesian_pos[3:])
#     print("Pos: ", pos)
#     param = rm_inverse_kinematics_params_t(current_joint, pos, flag=0)
# else:
