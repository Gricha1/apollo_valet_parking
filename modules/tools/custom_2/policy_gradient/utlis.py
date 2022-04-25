import torch
import json
import os
import numpy as np
from modules.tools.custom_2.EnvLib.ObstGeomEnv import *

def save_configs(config, folder_path, name):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    with open(f'{folder_path}/{name}', 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=4)

def save_weights(folder_path, model_weights):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    torch.save(model_weights, f'{folder_path}/policy.pkl')
 

def validate_task(env, agent, max_steps=800, idx=None, save_image=False, val_key=None):
    agent.config["explore"] = False
    observation = env.reset(idx=idx, fromTrain=False, val_key=val_key)
    images = []
    states = []
    accs = []
    phis = []
    #debug times
    times_action = []
    times_step = []

    if agent.config["model"]["use_lstm"]:
        prev_action = list(torch.zeros((2)))
        state = list(torch.zeros((2,256)))
    collision = False
    sum_reward = 0
    min_distance = float('inf')
    
    if save_image:
        images.append(env.render(sum_reward))

    isDone = False
    t = 0

    while not isDone and t < max_steps:
        #debug time
        start_time_action = time.time()

        if t % 10 == 0:
            print("steps:", t, end=" ")

        #get trajectory data for Apollo
        states.append(env.current_state)
        #accs.append(env.vehicle.a)
        if t != 0: #update t - 1
            Dx = states[t].x - states[t - 1].x
            Dy = states[t].y - states[t - 1].y
            if Dx != 0:
                phis.append(np.arctan(Dy / Dx))
            else:
                phis.append(1.5)
        if agent.config["model"]["use_lstm"]:
            action, state, logits = agent.compute_single_action(observation, state=state, prev_action=prev_action)
            prev_action = action
        else:
            action = agent.compute_single_action(observation)

        #clip and append action
        a = action[0]
        if env.vehicle.use_clip:
            a = np.clip(a, -env.vehicle.max_acc, env.vehicle.max_acc)
        if env.vehicle.is_jerk:
            if abs(a - env.vehicle.a) > env.vehicle.jerk:
                if a > env.vehicle.a:
                    a = env.vehicle.a + abs(a - env.vehicle.a)
                else:
                    a = env.vehicle.a - abs(a - env.vehicle.a)
        accs.append(a)
        #accs.append(env.vehicle.a)
        #accs.append(action[0])
            
        #debug time
        end_time_action = time.time()
        times_action.append(end_time_action - start_time_action)

        #debug time
        start_time_step = time.time()
        observation, reward, isDone, info = env.step(action)
        # print(f'info: {info}')
        if "EuclideanDistance" in info:
            if t % 10 == 0:
                print("dist to goal:", info["EuclideanDistance"], end = " ")
            if min_distance >= info["EuclideanDistance"]:
                min_distance = info["EuclideanDistance"]

        sum_reward += reward
        if save_image:
            images.append(env.render(sum_reward))
        t += 1

        end_time_step = time.time()
        times_step.append(end_time_step - start_time_step)
        if t % 10 == 0:
            print()

        if "SoftEps" in info or "Collision" in info:
            if "Collision" in info:
                collision = True
                isDone = False
            print("done:", isDone, "collision:", collision)
            break

    if save_image:
        images = np.transpose(np.array(images), axes=[0, 3, 1, 2])

    agent.config["explore"] = True


    Dx = states[-1].x - states[-2].x
    Dy = states[-1].y - states[-2].y
    if Dx != 0:
        phis.append(np.arctan(Dy / Dx))
    else:
        phis.append(1.5)

    return isDone, images, min_distance, \
            collision, [states, accs, phis, times_action, times_step]

def validation(env, agent):
    val_done_rate = 0
    min_total_val_dist = 0
    val_counter_collision = 0
    n_vals = 0
    for key in env.valTasks:
        # print(f'len(env.valTasks[key]): {len(env.valTasks[key])}')
        for i in range(len(env.valTasks[key])):
            # print(key)
            isDone, val_traj, min_distance, collision = validate_task(env, agent, idx=i, save_image=False, val_key=key)
            # print(f'collision: {collision}')
            # print(f'isDone: {isDone}')
            val_counter_collision += int(collision)
            val_done_rate += int(isDone)
            min_total_val_dist += min_distance 
        n_vals += len(env.valTasks[key])

    if n_vals < 1:
        n_vals = 1
    val_done_rate /= n_vals
    val_done_rate *= 100
    min_total_val_dist /= n_vals
    val_counter_collision /= n_vals
    val_counter_collision *= 100

    return val_done_rate, min_total_val_dist, val_counter_collision, val_traj

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
    

def generateTasks(config, buttom_road_edge_y, road_width, dynamic, union):
    min_dist = config['min_dist']
    max_dist = config['max_dist']
    max_val_vel = config['max_vel']
    alpha = config['alpha']
    discrete_alpha = config['discrete_alpha']
    max_steer = config['max_steer']
    alpha = degToRad(alpha)
    alphas = np.linspace(-alpha, alpha, discrete_alpha)
    valTasks = []

    if not dynamic and not union:
        empty_static = False
        empty_var_position = False
        empty_var_rotation = True
    else:
        empty_static = False
        empty_var_position = False
        empty_var_rotation = False


    #generate tasks
    #init point forward task
    forward_min_x, forward_max_x = 0, 10
    forward_min_y, forward_max_y = buttom_road_edge_y + 1.5, buttom_road_edge_y + road_width - 1.5
    forward_diff_x = forward_max_x - forward_min_x
    forward_diff_y = forward_max_y - forward_min_y

    #init point backward task  
    backward_min_x, backward_max_x = 22, 25    
    backward_min_y, backward_max_y = buttom_road_edge_y + 2.5, buttom_road_edge_y + road_width + road_width / 2
    backward_diff_x = backward_max_x - backward_min_x
    backward_diff_y = backward_max_y - backward_min_y
    theta_eps_ego = degToRad(15)
    speed_eps_ego = 5
    steer_eps_ego = degToRad(30)

    samples_theta_eps_ego = np.linspace(-theta_eps_ego, theta_eps_ego, 30)
    samples_speed_eps_ego_ = np.linspace(0, speed_eps_ego, 30)
    samples_steer_eps_ego = np.linspace(-steer_eps_ego, steer_eps_ego, 30)

    dyn_speed = np.linspace(0.5, 2, 30)
    for i in range(30):
        backward_x = backward_min_x + int(np.random.random() * backward_diff_x)
        backward_y = backward_min_y + int(np.random.random() * backward_diff_y)
        forward_x = forward_min_x + int(np.random.random() * forward_diff_x)
        forward_y = forward_min_y + int(np.random.random() * forward_diff_y)
        sample_theta_eps_ego = np.random.choice(samples_theta_eps_ego)
        sample_speed_eps_ego_ = np.random.choice(samples_speed_eps_ego_)
        sample_steer_eps_ego = 0
        #no union no dynamic
        dy_forward_start = max(1.5, np.random.random() * road_width)
        dy_forward_end = max(2, np.random.random() * (road_width + 0.5 * road_width))

        if not union:
            if dynamic:
                
                #generate forward task
                dyn_obs = [backward_x + 3, buttom_road_edge_y + road_width + 0.5 * road_width, 
                          degToRad(180), -dyn_speed[i], 0]
                valTasks.append(([forward_x, buttom_road_edge_y + dy_forward_start, 0, 0., 0], 
                                [backward_x, buttom_road_edge_y + road_width + 0.5 * road_width,
                                sample_theta_eps_ego, 0, 0], 
                                [dyn_obs]))

                #generate backward task
                dyn_obs = [forward_x - 1, buttom_road_edge_y + road_width + 0.5 * road_width, 
                           0, dyn_speed[i], 0]
                valTasks.append(([backward_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                                sample_theta_eps_ego, 0., 0], 
                                 [13, -5.5, degToRad(90), 0, 0], 
                                 [dyn_obs]))
                
                #generate forward task     
                dyn_obs = [backward_x + 3, buttom_road_edge_y + road_width + 0.5 * road_width, 
                          degToRad(180), -dyn_speed[i], 0]           
                valTasks.append(([forward_x, buttom_road_edge_y + dy_forward_start, 0, 0., 0], 
                                [backward_x, buttom_road_edge_y + dy_forward_end, 
                                sample_theta_eps_ego, 0, 0], 
                                [dyn_obs]))

                #generate backward task
                dyn_obs = [forward_x - 1, buttom_road_edge_y + road_width + 0.5 * road_width, 
                           0, dyn_speed[i], 0]
                valTasks.append(([backward_x, buttom_road_edge_y + dy_forward_end, 
                                sample_theta_eps_ego, 0, 0], 
                                 [13, -5.5, degToRad(90), 0, 0], 
                                 [dyn_obs]))

                
            else: #not union not dynamic
                if empty_static:
                    #generate forward task
                    valTasks.append(([forward_max_x, buttom_road_edge_y + 0.5 * road_width, 0, 0., 0], 
                                    [backward_min_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                                    0, 0, 0]))
                    #generate backward task
                    valTasks.append(([backward_max_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                                    0, 0, 0], 
                                     [13, -5.5, degToRad(90), 0, 0]))

                elif empty_var_position:
                    #generate forward task
                    valTasks.append(([forward_x, buttom_road_edge_y + dy_forward_start, 
                                    0, 0., 0], 
                                    [backward_x, buttom_road_edge_y + dy_forward_end, 
                                    0, 0, 0]))
                    #generate backward task
                    valTasks.append(([backward_x, buttom_road_edge_y + dy_forward_end, 
                                    0, 0, 0], 
                                    [13, -5.5, degToRad(90), 0, 0]))

                elif empty_var_rotation:
                    #generate forward task                
                    valTasks.append(([forward_x, buttom_road_edge_y + dy_forward_start, 0, 0., 0], 
                                    [backward_x, buttom_road_edge_y + dy_forward_end, 
                                    sample_theta_eps_ego, 0, 0]))
                    #generate backward task
                    valTasks.append(([backward_x, buttom_road_edge_y + dy_forward_end, 
                                    sample_theta_eps_ego, 0, 0], 
                                     [13, -5.5, degToRad(90), 0, 0]))

        else: #union task(with dynamic obsts)
            if dynamic:
                #generate forward task
                dyn_obs = [backward_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                        degToRad(180), dyn_speed[i], 0]
                valTasks.append(([forward_x, buttom_road_edge_y + 0.5 * road_width, 
                                0, 0., 0], 
                                [backward_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                                0, 0, 0], 
                                [dyn_obs]))
            else:
                valTasks.append(([forward_x, buttom_road_edge_y + 0.5 * road_width, 
                                0, 0., 0], 
                                [backward_x, buttom_road_edge_y + road_width + 0.5 * road_width, 
                                0, 0, 0]))


    return valTasks