import math
import random

import matplotlib.pyplot as plt
import numpy as np
import gym
import os
import json
from policy_gradient.ppo.ppo import PPO
from EnvLib.GeomEnv1 import *

show_animation = True
mark_size = 4
line_size = 2

class Transformation():
    def __init__(self): 
        self.diff_x = 0
        self.diff_y = 0
        self.theta = 0
        self.cos_theta = 0
        self.sin_theta = 0

    def rotate(self, start, goal):
        x_start = start[0]
        y_start = start[1]
        theta_start = start[2]
        x_goal = goal[0]
        y_goal = goal[1]
        theta_goal = goal[2]
        self.theta = math.atan2(y_goal - y_start, x_goal - x_start)
        self.diff_x = x_start
        self.diff_y = y_start
        x_start = 0
        y_start = 0
        x_goal -= self.diff_x
        y_goal -= self.diff_y
        self.cos_theta = math.cos(self.theta)
        self.sin_theta = math.sin(self.theta)
        new_x_goal = self.cos_theta * x_goal + self.sin_theta * y_goal
        new_y_goal = - self.sin_theta * x_goal + self.cos_theta * y_goal
        theta_start -= self.theta
        theta_goal -= self.theta

        return [x_start, y_start, theta_start], [new_x_goal, new_y_goal, theta_goal]
        
    def inverseRotate(self, x, y, theta):
        new_x = self.cos_theta * x - self.sin_theta * y
        new_y = self.sin_theta * x + self.cos_theta * y
        new_x += self.diff_x
        new_y += self.diff_y
        theta += self.theta
        return new_x, new_y, theta

with open("./configs/net_configs.json", 'r') as f:
    model_config = json.load(f)

with open('./configs/train_obst_configs.json', 'r') as f:
    train_config = json.load(f)


# valTasks = None

XYBounds = None

obstacle_map = []

XYBounds = [
    [-500, 500],
    [-500, 500]
]

obstacle_map = [
    [15, -8, 5, 1.5, pi/2]
]

vehicle_config = VehicleConfig(3.8, 2, 2.5, 0.1)

env = Environment(model_config, train_config)

state_dim = env.observation_space.shape[0]

action_dim = env.action_space.shape[0]

agent = PPO(state_dim, action_dim, model_config)

def validateRRT(agent, valTasks, obstacles, saveImage=False):
    
    # config["valTasks"] = valTasks
    max_steps = train_config["max_time_steps"]
    env.valTasks = valTasks     
    env.obstacle_map_init = obstacles
    
    state = env.reset(fromTrain=False)
    lst_params = []
    lst_params.append(env.current_state)
    isDone = False
    t = 0

    while not isDone and t < max_steps:
        action = agent.get_action(state, deterministic=True)
        state, reward, isDone, info = env.step(action, soft=False)
        lst_params.append(env.current_state)
        t += 1

        if "SoftEps" in info:
            break
        
    env.close()

    return isDone, lst_params


# valTasks = [([20., 20., degToRad(-135), kmToM(0)], [10, 10., degToRad(-135), kmToM(10)])]

# fig, ax = plt.subplots(figsize=(5, 5))

# lst_x = [valTasks[0][0][0], valTasks[0][1][0]]
# lst_y = [valTasks[0][0][1], valTasks[0][1][1]]
# plt.plot(lst_x, lst_y, 'b-', linewidth=line_size)
# ax.arrow(valTasks[0][0][0], valTasks[0][0][1], cos(valTasks[0][0][2]), sin(valTasks[0][0][2]), head_width=0.5, color='red', linewidth=line_size + 1)
# ax.arrow(valTasks[0][1][0], valTasks[0][1][1], cos(valTasks[0][1][2]), sin(valTasks[0][1][2]), head_width=0.5, color='cyan', linewidth=line_size + 1)

# transform = Transformation()
# start_transform, goal_transform = transform.rotate(valTasks[0][0][:3], valTasks[0][1][:3])
# print(f"start_transform: {start_transform}")
# print(f"goal_transform: {goal_transform}")
# # print(f"{self.alpha} : {self.alpha}")

# lst_x, lst_Y = [], []
# lst_x = [start_transform[0], goal_transform[0]]
# lst_y = [start_transform[1], goal_transform[1]]
# plt.plot(lst_x, lst_y, 'g-', linewidth=line_size)

# start_transform.append(valTasks[0][0][3])
# goal_transform.append(valTasks[0][1][3])

# valTasks = [(start_transform, goal_transform)]
# video, lst_params = validateRRT(valTasks, agent, config_defaults['max_timesteps'])

# ax.arrow(lst_params[0][0], lst_params[0][1], cos(lst_params[0][2]), sin(lst_params[0][2]), head_width=0.5, color='red', linewidth=line_size + 1)
# ax.arrow(lst_params[-1][0], lst_params[-1][1], cos(lst_params[-1][2]), sin(lst_params[-1][2]), head_width=0.5, color='cyan', linewidth=line_size + 1)

# lst_x, lst_y = [], []
# lst_x1, lst_y1 = [], []
# for i in range(len(lst_params)):
#     lst_x1.append(lst_params[i][0])
#     lst_y1.append(lst_params[i][1])
#     x, y, theta = transform.inverseRotate(lst_params[i][0], lst_params[i][1], lst_params[i][2])
#     lst_x.append(x)
#     lst_y.append(y)

# plt.plot(lst_x1, lst_y1, 'gH', markersize=mark_size)
# plt.plot(lst_x, lst_y, 'bH', markersize=mark_size)
# plt.show()
