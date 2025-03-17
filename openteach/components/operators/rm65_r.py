import numpy as np
import matplotlib.pyplot as plt
import zmq

from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R
from numpy.linalg import pinv

from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from openteach.robot.rm65_l import RM65L
from .operator import Operator

np.set_printoptions(precision=2, suppress=True)


class BimanualRightArmOperator(Operator):