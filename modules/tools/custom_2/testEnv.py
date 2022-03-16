import json
import ray.rllib.agents.ppo as ppo
import wandb
import torch
from EnvLib.ObstGeomEnv import *
from planning.generateMap import *

with open("configs/train_configs.json", 'r') as f:
    # train mode config
    train_config = json.load(f)

with open("configs/environment_configs.json", 'r') as f:
    # config for our env
    our_env_config = json.load(f)

with open('configs/reward_weight_configs.json', 'r') as f:
    reward_config = json.load(f)

with open('configs/car_configs.json', 'r') as f:
    car_config = json.load(f)

def validate_task(env, agent, max_steps=250, idx=None, save_image=False, val_key=None):
    state = env.reset(idx=idx, fromTrain=False, val_key=val_key)
    images = []
    collision = False
    sum_reward = 0
    min_distance = float('inf')
    
    if save_image:
        images.append(env.render(sum_reward))

    isDone = False
    t = 0

    while not isDone and t < max_steps:
        action = agent.compute_action(state)
        state, reward, isDone, info = env.step(action)
        # print(f'info: {info}')
        if "EuclideanDistance" in info:
            if min_distance >= info["EuclideanDistance"]:
                min_distance = info["EuclideanDistance"]

        sum_reward += reward
        if save_image:
            images.append(env.render(sum_reward))
        t += 1

        if "SoftEps" in info or "Collision" in info:
            if "Collision" in info:
                collision = True
            break

    if save_image:
        images = np.transpose(np.array(images), axes=[0, 3, 1, 2])

    return isDone, images, min_distance, collision

def validation(env, agent):
    val_done_rate = 0
    min_total_val_dist = 0
    val_counter_collision = 0
    n_vals = 0
    for key in env.valTasks:
        # print(f'len(env.valTasks[key]): {len(env.valTasks[key])}')
        for i in range(len(env.valTasks[key])):
            # print(key)
            isDone, _, min_distance, collision = validate_task(env, agent, idx=i, save_image=False, val_key=key)
            val_counter_collision += int(collision)
            val_done_rate += int(isDone)
            min_total_val_dist += min_distance 
        n_vals += len(env.valTasks[key])

    if not n_vals:
        n_vals = 1
    val_done_rate /= n_vals
    val_done_rate *= 100
    min_total_val_dist /= n_vals
    val_counter_collision /= n_vals
    val_counter_collision *= 100

    return val_done_rate, min_total_val_dist, val_counter_collision

def generateValidateTasks(config):
    min_dist = config['min_dist']
    max_dist = config['max_dist']
    max_val_vel = config['max_vel']
    alpha = config['alpha']
    discrete_alpha = config['discrete_alpha']
    max_steer = config['max_steer']
    alpha = degToRad(alpha)
    alphas = np.linspace(-alpha, alpha, discrete_alpha)
    valTasks = []

    for angle in alphas:
        for angle1 in alphas:
            valTasks.append(([0., 0., angle, 0., degToRad(np.random.randint(-max_steer, max_steer + 1))], [np.random.randint(min_dist, max_dist + 1), 0., angle1, kmToM(np.random.randint(0, max_val_vel + 1)), 0.]))

    return valTasks

valTasks = generateValidateTasks(our_env_config)

if our_env_config["obstacles"]:
    visualize_tasks = False
    maps = {}
    valTasks = {}
    trainTask = {}
    for index in range(12):
        maps["map" + str(index)] = readObstacleMap("maps/obstacle_map" + str(index)+ ".txt")
        valTasks["map" + str(index)] = readTasks("maps/val_map" + str(index)+ ".txt")
        trainTask["map" + str(index)] = readTasks("maps/train_map" + str(index)+ ".txt")
        # maps["map" + str(index)] = readObstacleMap("maps/obstacle_map" + str(index)+ ".txt")
        # valTasks["map" + str(index)] = readDynamicTasks("maps/dyn_train_map" + str(index)+ ".txt")
        # trainTask["map" + str(index)] = readDynamicTasks("maps/dyn_train_map" + str(index)+ ".txt")
        # if visualize_tasks:
        #     val_tasks = valTasks["map" + str(index)]
        #     train_task = trainTask["map" + str(index)]
        #     new_lst_bb = maps["map" + str(index)]
        #     plt.figure(figsize=(10, 10))
        #     for task in train_task:
        #         plt.plot(task[0][0], task[0][1], "rH")
        #         plt.plot(task[1][0], task[1][1], "cH")
        #         plt.plot([task[0][0], task[1][0]], [task[0][1], task[1][1]], '--r')
                
        #         bb_start = getBB([task[0][0], task[0][1], task[0][2]], ego=True)
        #         plt.plot([bb_start[(i + 1) % len(bb_start)][0] for i in range(len(bb_start) + 1)], [bb_start[(i + 1) % len(bb_start)][1] for i in range(len(bb_start) + 1)], '-r')
        #         plt.arrow(task[0][0], task[0][1], 2 * math.cos(task[0][2]), 2 * math.sin(task[0][2]), head_width=0.5, color='red')
                

        #         bb_goal = getBB([task[1][0], task[1][1], task[1][2]], ego=True)
        #         plt.plot([bb_goal[(i + 1) % len(bb_goal)][0] for i in range(len(bb_goal) + 1)], [bb_goal[(i + 1) % len(bb_goal)][1] for i in range(len(bb_goal) + 1)], '-c')
        #         plt.arrow(task[1][0], task[1][1], 2 * math.cos(task[1][2]), 2 * math.sin(task[1][2]), head_width=0.5, color='cyan')

        #         for new_bb in new_lst_bb:
        #             bbObs = getBB([new_bb[0], new_bb[1], new_bb[2]], w=new_bb[3], l=new_bb[4], ego=False)
        #             plt.plot([bbObs[(i + 1) % len(bbObs)][0] for i in range(len(bbObs) + 1)], [bbObs[(i + 1) % len(bbObs)][1] for i in range(len(bbObs) + 1)], '-b')
        #         plt.grid(True)
        #     plt.show()
else:
    maps = {"map1": []}
    trainTask = {"map1": []}
    valTasks = {"map1": valTasks}

obstacle_map = []
vehicle_config = VehicleConfig(car_config)

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
config['lr'] = train_config['lr']
config['sgd_minibatch_size'] = train_config['sgd_minibatch_size']
config['num_sgd_iter'] = train_config['num_sgd_iter']
config['clip_param'] = train_config['clip_param']
config["optimizer"] = train_config["optimizer"]
config['entropy_coeff'] = train_config['entropy_coeff']
config['vf_clip_param'] = train_config['vf_clip_param']
config['normalize_actions'] = train_config['normalize_actions']
config['batch_mode'] = train_config['batch_mode']
config['env_config'] = environment_config
    
env = ObsEnvironment(environment_config)
state_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]

state = env.reset()
for i in range(1, 100):
    if (not (i + 1) % 30):
        state = env.reset()
    print("################################")
    state, reward, done, info = env.step(np.array([40, 0]))     
    # print(f"state {state}")
    print(info)
     # env.weakenRestrictions()
    env.render(100, save_image=False)
    