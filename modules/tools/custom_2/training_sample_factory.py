"""
From the root of Sample Factory repo this can be run as:
python -m sample_factory_examples.train_custom_env_custom_model --algo=APPO --env=my_custom_env_v1 --experiment=example --save_every_sec=5 --experiment_summaries_interval=10
After training for a desired period of time, evaluate the policy by running:
python -m sample_factory_examples.enjoy_custom_env_custom_model --algo=APPO --env=my_custom_env_v1 --experiment=example
"""

from pickle import TRUE
import sys
from argparse import Namespace
import gym
import numpy as np
from torch import nn

from sample_factory.algorithms.utils.arguments import arg_parser, parse_args
# from sample_factory.algorithms.utils.pytorch_utils import calc_num_elements
from sample_factory.envs.env_registry import global_env_registry
from sample_factory.run_algorithm import run_algorithm
from EnvLib.ObstGeomEnvSampleFactory import *

def custom_parse_args(argv=None, evaluation=False):
    """
    Parse default SampleFactory arguments and add user-defined arguments on top.
    Allow to override argv for unit tests. Default value (None) means use sys.argv.
    Setting the evaluation flag to True adds additional CLI arguments for evaluating the policy (see the enjoy_ script).
    """
    parser = arg_parser(argv, evaluation=evaluation)

    # add custom args here
    # parser.add_argument('--my_custom_arg', type=int, default=42, help='Any custom arguments users might define')

    # SampleFactory parse_args function does some additional processing (see comments there)
    cfg = parse_args(argv=argv, evaluation=evaluation, parser=parser)
    return cfg

def make_custom_env_func(full_env_name, cfg=None, env_config=None):
    return ObsEnvironment(full_env_name, cfg)


def add_extra_params_func(env, parser):
    """
    Specify any additional command line arguments for this family of custom environments.
    """
    p = parser
    p.add_argument('--custom_env_num_actions', default=10, type=int, help='Number of actions in my custom env')
    p.add_argument('--custom_env_episode_len', default=250, type=int, help='Number of steps in the episode')


# def override_default_params_func(env, parser):
#     """
#     Override default argument values for this family of environments.
#     All experiments for environments from my_custom_env_ family will have these parameters unless
#     different values are passed from command line.
#     """
#     parser.set_defaults(
#         encoder_custom='custom_env_encoder',
#         hidden_size=128,
#     )


def register_custom_components():
    global_env_registry().register_env(
        env_name_prefix='polamp_env',
        make_env_func=make_custom_env_func,
        add_extra_params_func=add_extra_params_func,
        # override_default_params_func=override_default_params_func,
    )

import json
from policy_gradient.curriculum_train import generateDataSet
import ray.rllib.agents.ppo as ppo

with open("configs/train_configs.json", 'r') as f:
    train_config = json.load(f)

with open("configs/environment_configs.json", 'r') as f:
    our_env_config = json.load(f)

with open("configs/reward_weight_configs.json", 'r') as f:
    reward_config = json.load(f)



import ray.rllib.agents.ppo as ppo

def main():
    # dataSet = generateDataSet(our_env_config)
    # maps, trainTask, valTasks = dataSet["empty"]
    # maps_obst, trainTask_obst, valTasks_obst = dataSet["obstacles"]
    # maps_dyn_obst, trainTask_dyn_obst, valTasks_dyn_obst = dataSet["dyn_obstacles"]
    
    # if not our_env_config["empty"]:
    #     maps = maps_obst
    #     trainTask = trainTask_obst
    #     valTasks = valTasks_obst
    # if not our_env_config["obstacles"]:
    #     maps = maps_dyn_obst
    #     trainTask = trainTask_dyn_obst
    #     valTasks = valTasks_dyn_obst

    # environment_config = {
    #     'tasks': trainTask,
    #     'valTasks': valTasks,
    #     'maps': maps,
    #     'our_env_config' : our_env_config,
    #     'reward_config' : reward_config
    # }

    # config['env_config'] = environment_config
    print("1 =============== 1")
    """Script entry point."""
    register_custom_components()
    print("2 =============== 2")
    cfg = custom_parse_args()
    # print("type(cfg): ", type(cfg))
    print("3 =============== 3")
    # cfg.other_keys = environment_config
    # print("cfg: ", cfg)
    # all_config = Namespace(**cfg,
    #                         **environment_config)
    status = run_algorithm(cfg)
    # status = True
    return status


if __name__ == '__main__':
    sys.exit(main())