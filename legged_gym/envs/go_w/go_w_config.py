from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot
from legged_gym import LEGGED_GYM_ROOT_DIR
# from .mixed_terrains.anymal_c_rough_config import AnymalCRoughCfg
from .mixed_terrains.go_w_ice_config import *

class go_wheeled(LeggedRobot):
    cfg : GoIceCfg