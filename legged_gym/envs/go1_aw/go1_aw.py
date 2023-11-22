from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot
from .go1_aw_config import Go1AwFlatCfg


class Go1Aw(LeggedRobot):
    def _create_envs(self):
        # # add for wheel robot *************************************************************************************
        # self.num_rollers = 2
        super()._create_envs()


    def _init_buffers(self):
        # # add for wheel robot *************************************************************************************
        # self.num_rollers = 2
        super()._init_buffers()
        self.base_pos = self.root_states[:self.num_envs, 0:3]

    def _reward_jump(self):
        # penalize the height
        reference_heights = 0
        body_height = self.base_pos[:, 2] - reference_heights
        jump_height_target = self.commands[:, 3] + Go1AwFlatCfg.rewards.base_height_target
        reward = - torch.square(body_height - jump_height_target)
        return reward


    def _reward_cost_of_transport(self):
        # Mask out the joint you want to exclude
        mask = torch.ones(self.torques.size(-1), device=self.torques.device, dtype=torch.bool)
        mask[self.dof_roller_ids] = False  # This will work if joint_index_to_exclude is a list or tensor of indices

        # Apply the mask to torques and dof_vel
        masked_torques = self.torques[:, mask]
        masked_dof_vel = self.dof_vel[:, mask]


        # calculate energy self.dof_roller_ids
        power = torch.mul(masked_torques, masked_dof_vel)
        energy = power.sum(dim=-1) * self.dt
        # estimate distance
        # Correctly combine the linear and angular velocities into a single tensor
        velocities = torch.cat((self.base_lin_vel[:, :2], self.base_ang_vel[:, 2].unsqueeze(-1)), dim=-1)
        distance = torch.norm(velocities, p=2, dim=-1) * self.dt + 1e-6
        
        # Calculate CoT, remove constant value, m and g
        cot = energy / distance
        # Ensure that cot has the same shape as self.rew_buf
        # cot = cot.view_as(self.rew_buf)
        return cot
    


