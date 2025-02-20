import numpy as np
import time
from copy import deepcopy as copy
from enum import Enum
import math

from openteach.constants import SCALE_FACTOR
from scipy.spatial.transform import Rotation as R