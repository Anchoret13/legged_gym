

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class Go1AwFlatCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 247
        num_actions = 16
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.3] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]

            'FL_roller_foot_joint': 0,
            'FR_roller_foot_joint': 0,
            'RL_roller_foot_joint': 0,
            'RR_roller_foot_joint': 0
        }
    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        # static_friction = 10.0
        # dynamic_friction = 1.0
        measure_heights = False
        

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'hip_joint': 20.0, 'thigh_joint': 25.0, 'calf_joint': 25.0, 'roller': 0.0}  # [N*m/rad]
        damping = {'hip_joint': 0.5, 'thigh_joint': 0.5, 'calf_joint': 0.5, 'roller': 0.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go1_aw/urdf/go1_aw.urdf'
        name = "go1"
        foot_name = "foot"
        roller_name = "roller"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
       
        default_dof_drive_mode = 3 # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        replace_cylinder_with_capsule = False # replace collision cylinders with capsules, leads to faster/more stable simulation
        flip_visual_attachments = False # Some .obj meshes must be flipped from y-up to z-up
        

    class commands(LeggedRobotCfg.commands):
        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [-0.3, 3.0] # min max [m/s]
            lin_vel_y = [-0.5, 1.0]   # min max [m/s]
            ang_vel_yaw = [-0.5, 0.5]    # min max [rad/s]
            heading = [-3.14/2, 3.14/2]

    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.30
        max_contact_force = 200
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = 0.0001
            tracking_lin_vel = 5.0
            tracking_ang_vel = 0.5
            dof_pos_limits = -10.0
            orientation = 1.0
            lin_vel_z = -0.02
            ang_vel_xy = -0.001

            feet_air_time = 1.0
            action_rate = -0.1
            # reward added by xiaoyu
            jump = 0.0
            cost_of_transport = -0.0001

class Go1AwFlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'go1_aw_flat'

  