import json
import wandb
import torch
import ray.rllib.agents.ppo as ppo
import ray.rllib.agents.ddpg as ddpg
from EnvLib.ObstGeomEnv import *
from planning.generateMap import *
from policy_gradient.utlis import *

with open("configs/environment_configs.json", 'r') as f:
    our_env_config = json.load(f)

ANGLE_EPS = float(our_env_config['ANGLE_EPS'])
SPEED_EPS = float(our_env_config['SPEED_EPS'])
STEERING_EPS = float(our_env_config['STEERING_EPS'])


def trasitionCurriculum(trainer, val_env, environment_config, config, current_counter):
    model_weights = trainer.get_policy().get_weights()
    environment_config['our_env_config']['ANGLE_EPS'] = ANGLE_EPS / (current_counter + 1)
    environment_config['our_env_config']['SPEED_EPS'] = SPEED_EPS / (current_counter + 1)
    environment_config['our_env_config']['STEERING_EPS'] = STEERING_EPS / (current_counter + 1)
    config['env_config'] = environment_config
    trainer = ppo.PPOTrainer(config=config, env=ObsEnvironment)
    trainer.get_policy().set_weights(model_weights)
    val_env = ObsEnvironment(environment_config)
    print("current_counter: ", current_counter)
    print(trainer.config['env_config']['our_env_config'])
    
    return trainer, val_env

def generateDataSet(our_env_config):
    #CUSTOM DATASET
    dataSet = {}
    maps = {} 
    trainTask = {}
    valTasks = {}
    if our_env_config["dynamic"]: dynamic = True
    else: dynamic = False
    if our_env_config["union"]: union = True
    else: union = False

    #left and right bottom boundaries
    bottom_left_boundary_center_x = 5 #4.5
    bottom_left_boundary_center_y = -5.5
    bottom_left_boundary_width = 2
    bottom_right_boundary_center_x = 21 #21.5
    bottom_right_boundary_center_y = -5.5
    bottom_right_boundary_width = 2
    #bottom boundary x change
    bottom_diff_x = 4
    #upper boundary
    buttom_road_edge_y = bottom_left_boundary_center_y + bottom_left_boundary_width
    road_width = 5
    center_line_y = (bottom_left_boundary_center_y + bottom_left_boundary_width) + road_width
    upper_boundary_width = 2
    upper_boundary_center_y = center_line_y + road_width + upper_boundary_width

    for index in range(5):
        bottom_dx = np.random.random() * bottom_diff_x

        if our_env_config["static"]:
            maps["map" + str(index)] = [[13, upper_boundary_center_y, 0, upper_boundary_width, 17], 
                                        [bottom_left_boundary_center_x - bottom_dx, bottom_left_boundary_center_y, 0, 
                                        bottom_left_boundary_width, 6],
                                        [bottom_right_boundary_center_x + bottom_dx, bottom_right_boundary_center_y, 0,
                                        bottom_right_boundary_width, 6], 
                                        [13, -8.5, 0, 1, 17]]
        else:
            maps["map" + str(index)] = []

        trainTask["map" + str(index)] = generateTasks(our_env_config, buttom_road_edge_y, road_width, 
                                                    dynamic=dynamic, union=union)
        valTasks["map" + str(index)] = generateTasks(our_env_config, buttom_road_edge_y, road_width, 
                                                    dynamic=dynamic, union=union)
        road_width += 0.5
        buttom_road_edge_y = bottom_left_boundary_center_y + bottom_left_boundary_width
        center_line_y = (bottom_left_boundary_center_y + bottom_left_boundary_width) + road_width
        upper_boundary_width = 2
        upper_boundary_center_y = center_line_y + road_width + upper_boundary_width

    dataSet["empty"] = (maps, trainTask, valTasks)
    
    maps_obst = {}
    valTasks_obst = {}
    trainTask_obst = {}
    for index in range(12):
        maps_obst["map" + str(index)] = readObstacleMap("maps/obstacle_map" + str(index)+ ".txt")
        valTasks_obst["map" + str(index)] = readTasks("maps/val_map" + str(index)+ ".txt")
        trainTask_obst["map" + str(index)] = readTasks("maps/train_map" + str(index)+ ".txt")
    dataSet["obstacles"] = (maps_obst, trainTask_obst, valTasks_obst)

    maps_dyn_obst = {}
    valTasks_dyn_obst = {}
    trainTask_dyn_obst = {}
    for index in range(12):
        maps_dyn_obst["map" + str(index)] = readObstacleMap("maps/obstacle_map" + str(index)+ ".txt")
        valTasks_dyn_obst["map" + str(index)] = readDynamicTasks("maps/dyn_val_map" + str(index)+ ".txt")
        trainTask_dyn_obst["map" + str(index)] = readDynamicTasks("maps/dyn_train_map" + str(index)+ ".txt")
    dataSet["dyn_obstacles"] = (maps_dyn_obst, trainTask_dyn_obst, valTasks_dyn_obst)

    # print(f"dataSet {dataSet}")
    return dataSet

def trainCurriculum(config, train_config, our_env_config, reward_config, car_config, ppo_algorithm=True):
    
    dataSet = generateDataSet(our_env_config)
    maps, trainTask, valTasks = dataSet["empty"]
    maps_obst, trainTask_obst, valTasks_obst = dataSet["obstacles"]
    maps_dyn_obst, trainTask_dyn_obst, valTasks_dyn_obst = dataSet["dyn_obstacles"]
    
    vehicle_config = VehicleConfig(car_config)

    if not our_env_config["empty"]:
        maps = maps_obst
        trainTask = trainTask_obst
        valTasks = valTasks_obst
    if not our_env_config["obstacles"]:
        maps = maps_dyn_obst
        trainTask = trainTask_dyn_obst
        valTasks = valTasks_dyn_obst

    environment_config = {
        'vehicle_config': vehicle_config,
        'tasks': trainTask,
        'valTasks': valTasks,
        'maps': maps,
        'our_env_config' : our_env_config,
        'reward_config' : reward_config
    }

    config['env_config'] = environment_config


    if ppo_algorithm:
        trainer = ppo.PPOTrainer(config=config, env=ObsEnvironment)
    else:
        trainer = ddpg.DDPGTrainer(config=config, env=ObsEnvironment)
    val_env = ObsEnvironment(environment_config)

    print(trainer.config["model"])
    print(trainer.config['env_config']['our_env_config'])

    folder_path = "./myModelWeight1"

    #load model
    #trainer.restore("./myModelWeight1/new_dynamic_steps_7/5_step_2/checkpoint_003580/checkpoint-3580")
    trainer.restore("./myModelWeight1/new_dynamic_steps_7/6_step_3/checkpoint_003700/checkpoint-3700")
    if train_config["curriculum"]:
        print("curriculum")
        curr_folder_path = os.path.join(folder_path, train_config['curriculum_name'])
        #trainer.get_policy().set_weights(torch.load(f'{curr_folder_path}/policy.pkl'))
        print("get folder_path: ", curr_folder_path)

    if train_config["curriculum"]:
        folder_path = os.path.join(folder_path, train_config['name_save'] + "Curriculum")
    else:
        folder_path = os.path.join(folder_path, train_config['name_save'])
    print("save folder_path: ", folder_path)

    save_configs(car_config, folder_path, "car_configs.json")
    save_configs(reward_config, folder_path, "reward_weight_configs.json")
    save_configs(train_config, folder_path, "train_configs.json")
    save_configs(our_env_config, folder_path, "environment_configs.json")

    t_max = train_config["max_episodes"]
    t_val = train_config["val_time_steps"]
    acceptable_success = train_config["acceptable_success"]
    flag_wandb = train_config["wandb"]
    if flag_wandb:
        wandb_config = dict(car_config, **train_config, **our_env_config, **reward_config)
        wandb.init(config=wandb_config, project=train_config['project_name'], entity='grisha1')
    t = 0
    old_val_rate = -1
    current_counter = 0
    old_step = 0

    only_show = True
    #time parametets
    if only_show: 
        t_show = 1
    else:
        t_show = 40 #40
        t_save = 10 #10
        t_validate = 5 #5

    while t <= (t_max * 250):
        print(f"t {t}", end=" ")
        t += 1

        if only_show:
            if t % t_show == 0:
                for key in val_env.valTasks:
                    _, val_traj, _, _ = validate_task(val_env, trainer, save_image=True, val_key=key)
                    if flag_wandb:
                        print("get video", end=" ")
                        wandb.log({f"Val_trajectory_{t}": wandb.Video(val_traj, fps=10, format="gif")})
            print()

        else:
            #train
            res = trainer.train()
            log = res['episode_reward_mean']
            info = res['info']['learner']['default_policy']['learner_stats']

            #get video
            if t % t_show == 0:
                for key in val_env.valTasks:
                    _, val_traj, _, _ = validate_task(val_env, trainer, 
                                                save_image=True, val_key=key)
                    if flag_wandb:
                        print("get video", end=" ")
                        wandb.log({f"Val_trajectory_{t}": wandb.Video(val_traj, 
                                                        fps=10, format="gif")})

            if flag_wandb:
                if ppo_algorithm:
                    wandb.log(
                            {
                            'episode_reward_mean': log,
                            "total_loss": info['total_loss'],
                            "vf_loss": info['vf_loss'],
                            "kl" : info['kl'],
                            "policy_loss": info['policy_loss'],
                            "entropy": info['entropy']
                            },
                            step = (res['info']['num_agent_steps_trained'] + old_step)
                            )
                else:
                    wandb.log(
                            {
                            'episode_reward_mean': log,
                            "actor_loss": info['actor_loss'],
                            "critic_loss": info['critic_loss'],
                            "mean_q" : info['mean_q']
                            },
                            step = (res['info']['num_agent_steps_trained'] + old_step)
                            )

                #get validate result
                if (t % t_validate == 0):
                    val_done_rate, min_total_val_dist, val_counter_collision = validation(val_env, 
                                                                                        trainer)
                    print(f"val_done_rate {val_done_rate}", end=" ")

                    if flag_wandb:
                        wandb.log(
                            {
                            'val_done_rate': val_done_rate,
                            "min_val_dist": min_total_val_dist,
                            "val_counter_collision": val_counter_collision
                            }, 
                            step = (res['info']['num_agent_steps_trained'] + old_step))
                
                #save the model
                name_of_adding_folder = "./myModelWeight1/" + "new_dynamic_steps_7/7_step_3"
                if t % t_save == 0:
                    print("save model", end = " ")
                    if not os.path.exists(name_of_adding_folder):
                        os.makedirs(name_of_adding_folder)
                    trainer.save(name_of_adding_folder)
                
            print()
