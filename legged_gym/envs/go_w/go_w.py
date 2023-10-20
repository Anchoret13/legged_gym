from time import time
import numpy as np
import os
import torch
from legged_gym.envs import LeggedRobot

class GO_W(LeggedRobot):
    def reward(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1