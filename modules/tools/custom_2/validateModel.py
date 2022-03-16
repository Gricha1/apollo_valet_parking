import json
import ray.rllib.agents.ppo as ppo
#import wandb
import torch
from modules.tools.custom_2.EnvLib.ObstGeomEnv import *
from modules.tools.custom_2.planning.generateMap import *
from modules.tools.custom_2.policy_gradient.utlis import *
from modules.tools.custom_2.generate_trajectory import create_task, generate_first_goal
import time
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

class point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


def get_points(roi_boundaries, vehicle_pos, parking_pos, max_steps=30):
    """
    input:
    roi_boundaries: list of type: point
    vehicle_pos: type: point
    parking_pos: list of type: float

    return:
    isDone, images, trajectory (list of type: point), info_
    """
    
    start = time.time()
   
    train_config = {
    "framework": "torch",
    "wandb" : 0,
    "train_batch_size": 8000,
    "lambda": 0.96,
    "use_critic": 1,
    "use_gae": 1,
    "horizon": 600,
    "rollout_fragment_length": 1600,
    "num_gpus": 1,
    "num_workers": 5,
    "lr": 0.0001,
    "sgd_minibatch_size": 512,
    "num_sgd_iter": 10,
    "clip_param": 0.2,
    "optimizer": "adam",
    "entropy_coeff": 0.01,
    "vf_clip_param": 100,
    "normalize_actions": 0,
    "batch_mode": "complete_episodes",
    "max_episodes": 100000,
    "val_time_steps": 250,
    "acceptable_success": 80,
    "name_save": "rllib_ppoNew3",
    "curriculum_name": "rllib_ppoWithOrientation",
    "project_name": "POLAMP_ppo",
    "curriculum": 0,
    "steps_curriculum": 1,
    "_fake_gpus": 1,
    "vf_loss_coeff": 0.1,
    "use_lstm": 0,
    "lstm_use_prev_action": 0,
    "free_log_std": 1
    }

    our_env_config = {
    "empty": 1,
    "obstacles": 1,
    "dyn_obstacles": 0,
    "alpha": 45,
    "discrete_alpha": 11,
    "max_steer": 10,
    "max_dist": 30,
    "min_dist": 15,
    "min_vel": 0,
    "max_vel": 10,
    "min_obs_v": 0,
    "max_obs_v": 0,
    "UPDATE_SPARSE": 8,
    "HARD_EPS": 0.5,
    "SOFT_EPS": 0.5,
    "ANGLE_EPS": 15,
    "SPEED_EPS": 6,
    "STEERING_EPS": 20,
    "view_angle": 180,
    "MAX_DIST_LIDAR": 20,
    "bias_beam": 0,
    "n_beams": 39,
    "frame_stack": 4,
    "hard_constraints": 0,
    "soft_constraints": 0,
    "affine_transform": 0,
    "reward_with_potential": 1,
    "union": 1,
    "dynamic": 0,
    "static": 1
    }


    reward_config = {
    "collision": 20,
    "goal": 200,
    "timeStep": 1,
    "distance": 3,
    "overSpeeding": 5,
    "overSteering": 5
    }  

    car_config = {
    "length": 4.933,
    "width": 2.11,
    "wheel_base": 2.8448,
    "safe_eps": 0.001,
    "max_steer": 28,
    "max_vel": 5,
    "min_vel": -5,
    "max_acc": 0.5,
    "max_ang_vel": 10,
    "max_ang_acc": 1,
    "delta_t": 0.1
    }
    

    rrt = True

    info_ = {}

    vehicle_config = VehicleConfig(car_config)
    info_["car_length"] = car_config["length"]
    info_["car_width"] = car_config["width"]
    info_["wheel_base"] = car_config["wheel_base"]

    #set the first goal
    first_goal = generate_first_goal(roi_boundaries, vehicle_pos, parking_pos)
    info_["first_goal"] = first_goal
    #create validate task
    maps, _, valTasks = create_task(roi_boundaries, vehicle_pos, 
                                                parking_pos, first_goal)

    #DEBUG
    #parking_pos = point(parking_pos[0] - car_config["wheel_base"] / 2,
    #                     parking_pos[1] - car_config["wheel_base"] / 2)
    parking_pos = point(parking_pos[0], parking_pos[1])
    print("parking_pose:", parking_pos.x, parking_pos.y)
    print()
    print()
    print()

    #set configuration
    second_goal = [parking_pos.x, parking_pos.y - 1.5, degToRad(90), 0, 0]
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
            'tasks': trainTask,
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


    folder_path = "./myModelWeight1"
    train_config['curriculum_name'] = "rllib_ppoWithOrientation"
    curriculum_name = train_config['curriculum_name']
    print(train_config["curriculum_name"])
    folder_path = os.path.join(folder_path, train_config['curriculum_name'])

    ##Load weights
    #new weights
    trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/checkpoint_003730/checkpoint-3730")
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/checkpoint_003960/checkpoint-3960")
    #old weights
    #trainer.restore("modules/tools/custom_2/myModelWeight1/new_weights/checkpoint_003130/checkpoint-3130")
        
    #ANGLE_EPS = float(our_env_config['ANGLE_EPS']) / 2.
    #SPEED_EPS = float(our_env_config['SPEED_EPS']) / 2.
    #STEERING_EPS = float(our_env_config['STEERING_EPS']) / 2.

    '''
    ##wandb using
    #wandb_config = dict(car_config, **train_config, **our_env_config, **reward_config)
    #wandb.init(config=wandb_config, project=train_config['project_name'], entity='grisha1')
    '''
    end = time.time()
    print("initialization_time: ", end - start)
    start = time.time()
    
    for key in val_env.valTasks:
        isDone, val_traj, _, _, states = validate_task(val_env,
                                 trainer, max_steps = max_steps,
                                 save_image=True, val_key=key)
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

