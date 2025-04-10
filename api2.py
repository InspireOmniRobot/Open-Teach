from pprint import pprint
import socket
from turtle import home
import numpy as np
import json
from Robotic_Arm.rm_robot_interface import *

# left: 192.168.2.18
LEFT_IP = "192.168.2.21"
# HOME_LEFT_POSE = [0.0875, -0.1998, -0.3511, -2.923, 0.034, 1.558]
# HOME_LEFT_JOINT = [140, 115, 45, 110, 20, -55]
HOME_LEFT_POSE = [0.201902, -0.264378, -0.213804, -1.55, 0.756, 2.25]
HOME_LEFT_JOINT = [-25, -98, -32, -54, -81, -63]

# right: 192.168.2.19
RIGHT_IP = "192.168.2.19"
# HOME_RIGHT_POSE = [-0.0875, -0.1998, -0.3511, -2.923, -0.034, -1.558]
# HOME_RIGHT_JOINT = [-140, -115, -45, -110, -20, 55]
HOME_RIGHT_POSE = [-0.201902, -0.264378, -0.213804, -1.55, -0.756, -2.25]
# HOME_RIGHT_POSE_QUAD = [
#     -0.2016877979040146,
#     -0.2639901638031006,
#     -0.21389810740947723,
#     -0.05336063355207443,
#     0.5180320143699646,
#     -0.47313812375068665,
#     0.7105883359909058,
# ]
HOME_RIGHT_JOINT = [25, 98, 32, 54, 81, 63]

HOME_RIGHT_JOINT_CANFD = [25000, 98000, 32000, 54000, 81000, 63000]

np.set_printoptions(precision=4, suppress=True)

# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm(RIGHT_IP, 8080, level=3)

result = arm.rm_get_current_tool_frame()
print(result)
# arm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# arm_socket.connect((RIGHT_IP, 8080))
# json_str = json.dumps(
#     {'command': 'movej', 'joint': [25000, 98000, 32000, 54000, 81000, 63000], 'v': 5, 'r': 0, 'trajectory_connect': 0}
# ).encode("utf-8")
# print(json_str)
# arm_socket.sendall(json_str)

# resp = arm_socket.recv(1024)
# print(resp)

# 初始化算法的机械臂及末端型号
algo_handle = Algo(
    rm_robot_arm_model_e.RM_MODEL_RM_65_E, rm_force_type_e.RM_MODEL_RM_B_E
)
algo_handle.rm_algo_set_redundant_parameter_traversal_mode(False)


print(algo_handle.rm_algo_get_curr_toolframe())

current_joint = arm.rm_get_joint_degree()[1]
param = rm_inverse_kinematics_params_t(current_joint, HOME_RIGHT_POSE, flag=1)
code, joint = algo_handle.rm_algo_inverse_kinematics(param)
print(code, joint)

# result = arm.rm_movej(HOME_RIGHT_JOINT, v=5, r=0, connect=0, block=1)
# print(result)
# print(code, result)
# print(algo_handle.rm_algo_forward_kinematics(HOME_RIGHT_JOINT, flag=1))
# arm.rm_movej(HOME_RIGHT_JOINT, v=5, r=0, connect=0, block=1)
# arm.rm_movej(HOME_RIGHT_JOINT, follow=False)

arm.rm_delete_robot_arm()
