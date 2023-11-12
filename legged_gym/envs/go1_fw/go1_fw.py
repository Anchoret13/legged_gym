# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot

class Go1Fw(LeggedRobot):
    def _create_envs(self):
        # # add for wheel robot *************************************************************************************
        # self.num_rollers = 2
        super()._create_envs()

    #     for i in range(self.num_envs):
    #         props = self.gym.get_actor_dof_properties(ref_env, actor_handle)
    #     self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

        # print('*******************************t***************************************************')
        # self.num_dof
        # print(self.num_dof)
        # print(self.num_bodies)
        # print(self.dof_names)
        # print(self.num_dof)
        # self.num_rollers = len(self.dof_names)
        # self.dof_names = [string for string in self.dof_names if "roller" not in string]
        # print(self.dof_names)
        # self.num_rollers = self.num_rollers - len(self.dof_names)
        # print(self.num_rollers)
        # self.num_dof = self.num_dof - self.num_rollers
        # print(self.feet_names )
        # print('**********************************************************************************')

        # # self.num_dofs = len(self.dof_names) - self.num_rollers
        # print(self.num_dofs )
        # # self.num_dofs = len(self.dof_names) - self.num_rollers




    # def reset_idx(self, env_ids):
    #     super()._create_envs(env_ids)
    #     self.roller_air_time[env_ids] = 0.

    # def _reward_roller_air_time(self):
    #     # Reward long steps
    #     # Need to filter the contacts because the contact reporting of PhysX is unreliable on meshes
    #     self.roller_names = [s for s in self.dof_names if 'roller' in s]
    #     self.roller_indices = [] 
    #     for i in range(len(self.roller_names)):
    #         self.roller_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0], self.actor_handles[0], self.roller_names[i])

    #     contact = self.contact_forces[:, self.self.feet_indices, 2] > 1.
    #     contact_filt = torch.logical_or(contact, self.last_contacts) 
    #     self.last_contacts = contact

    #     self.roller_air_time = torch.zeros(self.num_envs, self.roller_indices.shape[0], dtype=torch.float, device=self.device, requires_grad=False)

    #     first_contact = (self.roller_air_time > 0.) * contact_filt
    #     self.roller_air_time += self.dt
    #     rew_airTime = torch.sum((self.roller_air_time - 0.5) * first_contact, dim=1) # reward only on first contact with the ground
    #     rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1 #no reward for zero command
    #     self.roller_air_time *= ~contact_filt
    #     return rew_airTime


