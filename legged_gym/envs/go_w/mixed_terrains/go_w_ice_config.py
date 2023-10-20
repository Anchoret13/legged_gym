from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GoIceCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_actions = 12

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'ice'

    class init_state( LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.6]
        default_joint_angles = {} # TODO: tobe update

    class control(LeggedRobotCfg.control):
        # TODO: pass
        pass
    
    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROBOT_DIR}/resources/robots/"

    class domain_rand(LeggedRobotCfg.domain_rand):
        pass

    class rewards(LeggedRobotCfg.rewards):
        pass

class IcePPO(LeggedRobotCfgPPO):
    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'ice'
        load_run = -1