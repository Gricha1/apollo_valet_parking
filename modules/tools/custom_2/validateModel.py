import json
import ray.rllib.agents.ppo as ppo
#import wandb
import torch
#import pickle
import pickle5 as pickle
from modules.tools.custom_2.EnvLib.ObstGeomEnv import *
from modules.tools.custom_2.planning.generateMap import *
from modules.tools.custom_2.policy_gradient.utlis import *
from modules.tools.custom_2.generate_trajectory import create_task
from modules.tools.custom_2.update_config import \
                                            update_car_config, \
                                            update_our_env_config, \
                                            update_train_config, \
                                            update_reward_config
import time
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

class point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


def get_points(roi_boundaries, vehicle_pos, parking_pos, max_steps=30, dyn_obsts=[]):
    """
    input:
    roi_boundaries: list of type: point
    vehicle_pos: type: point
    parking_pos: list of type: float

    return:
    isDone, images, trajectory (list of type: point), info_
    """
    
    start = time.time()
    train_config = update_train_config()
    car_config = update_car_config()
    our_env_config = update_our_env_config()
    reward_config = update_reward_config()
    rrt = True
    info_ = {}
    vehicle_config = VehicleConfig(car_config)
    info_["car_length"] = car_config["length"]
    info_["car_width"] = car_config["width"]
    info_["wheel_base"] = car_config["wheel_base"]

    #create validate task
    maps, _, valTasks, second_goal = create_task(roi_boundaries, vehicle_pos, 
                                                parking_pos, dyn_obsts)
    info_["first_goal"] = valTasks["map0"][0][1]

    #DEBUG
    print("parking_pose:", parking_pos[0], parking_pos[1])
    print()
    print()

    #set configuration
    #second_goal = [parking_pos.x, parking_pos.y - 1.5, degToRad(90), 0, 0]
    info_["second_goal"] = second_goal
    if our_env_config["union"]:
        environment_config = {
            'vehicle_config': vehicle_config,
            'tasks': _,
            'valTasks': valTasks,
            'maps': maps,
            'our_env_config' : our_env_config,
            'reward_config' : reward_config,
            "second_goal" : second_goal
        }
    else:
        environment_config = {
            'vehicle_config': vehicle_config,
            'tasks': _,
            'valTasks': valTasks,
            'maps': maps,
            'our_env_config' : our_env_config,
            'reward_config' : reward_config
        }

    config = ppo.DEFAULT_CONFIG.copy()
    config['framework'] = train_config['framework']
    config['train_batch_size'] = train_config['train_batch_size']
    config['lambda'] = train_config['lambda']
    config['use_critic'] = train_config['use_critic']
    config['use_gae'] = train_config['use_gae']
    config['horizon'] = train_config['horizon']
    config['rollout_fragment_length'] = train_config['rollout_fragment_length']
    config['num_gpus'] = train_config['num_gpus']
    config['num_workers'] = train_config['num_workers']
    config['num_workers'] = 1
    config['lr'] = train_config['lr']
    config['sgd_minibatch_size'] = train_config['sgd_minibatch_size']
    config['num_sgd_iter'] = train_config['num_sgd_iter']
    config['clip_param'] = train_config['clip_param']
    config["optimizer"] = train_config["optimizer"]
    config['entropy_coeff'] = train_config['entropy_coeff']
    config['vf_clip_param'] = train_config['vf_clip_param']
    config['normalize_actions'] = train_config['normalize_actions']
    config['batch_mode'] = train_config['batch_mode']
    config['_fake_gpus'] = train_config['_fake_gpus']
    config['vf_loss_coeff'] =  train_config['vf_loss_coeff']
    config['env_config'] = environment_config
    config["explore"] = False

    config["model"]['fcnet_hiddens'] = [512, 512]
    config["model"]["use_lstm"] = train_config['use_lstm']
    config["model"]["lstm_use_prev_action"] = train_config['lstm_use_prev_action']
    config["model"]["free_log_std"] = train_config["free_log_std"]


    #print("###########till env, trainer")
    val_env = ObsEnvironment(environment_config)
    #print("########after env")
    trainer = ppo.PPOTrainer(config=config, env=ObsEnvironment)
    #print("##########after env, trainer")

    ##Load weights
    #new weights
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/checkpoint_003730/checkpoint-3730")
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/week10/checkpoint-4380")
    #old weights
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/ex_1/checkpoint_004830/checkpoint-4830")
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/ex_2/checkpoint_009190/checkpoint-9190")
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/ex_2/new_weights.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_30/weights_1_2_30.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_35/weights_1_2_35.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_40/weights_1_2_40.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_39/weights_1_2_39.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_35/weights_1_2_35.pickle', 'rb') as handle:
    #with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_39/weights_1_2_39.pickle', 'rb') as handle:
    with open('modules/tools/custom_2/myModelWeight1/new_weights/conf_1_ex_2_run_48/weights_1_2_48.pickle', 'rb') as handle:
        weights = pickle.load(handle)
        trainer.get_policy().set_weights(weights)

    end = time.time()
    print("initialization_time: ", end - start)
    start = time.time()
    
    for key in val_env.valTasks:
        isDone, val_traj, _, _, states = validate_task(val_env,
                                 trainer, max_steps = max_steps,
                                 save_image=False, val_key=key)
        '''
        ##wandb using
        #wandb.log({f"Val_trajectory_{0}": wandb.Video(val_traj, fps=10, format="gif")})
        '''

    end = time.time()

    print("algorithm_time: ", end - start)

    return isDone, val_traj, states, info_


def test_get_points():

    roi_boundaries = [point(0, 0), point(10, 0),
                        point(10, -5), point(14, -5), 
                        point(14, 0), point(24, 0),
                        point(24, 10), point(0, 10)]
    vehicle_pos = point(0, 2.5)
    parking_pos = point(6.5, -1.5)

    states = get_points(roi_boundaries, vehicle_pos, parking_pos)

    print([(state.x, state.y) for state in states])


if __name__ == "__main__":
    test_get_points()

