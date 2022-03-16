import matplotlib.pyplot as plt
import numpy as np
import math
import torch 
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy
from pickle import TRUE
from modules.tools.custom_2.planning.utilsPlanning import *
import time
from modules.tools.custom_2.planning.dwa_steering import planningDWA

def validate_task(env, agent, max_steps=250, idx=None, save_image=False, val_key=None, goal=False, dyn_trajectories=[]):
    dyn_obs_trajectories = list(dyn_trajectories)
    id_dyn_obst = 0
    idx = 0
    observation = env.reset(idx=idx, fromTrain=False, val_key=val_key, rrt=True)
    initial_distance = math.hypot(env.current_state.x - env.goal.x, env.current_state.y - env.goal.y)
    images = []
    if agent.config["model"]["use_lstm"]:
        prev_action = list(torch.zeros((2)))
        state = list(torch.zeros((2,256)))
    collision = False
    sum_reward = 0
    min_distance = float('inf')
    
    if save_image:
        images.append(env.render(sum_reward))
    else:
        images.append(env.current_state)
    isDone = False
    t = 0
    steering_time = 0
    id_dyn_obst = 0
    
    while not isDone and t < max_steps:
        id_dyn_obst += 1
        dyn_obstacles = []
        for dyn_obst in dyn_obs_trajectories:
            if id_dyn_obst < len(dyn_obst):
                dyn_obstacles.append(dyn_obst[id_dyn_obst])
            else:
                dyn_obstacles.append(dyn_obst[-1])
        # print(f"dyn_obstacles {dyn_obstacles}")
        # for dyn in env.dynamic_obstacles:
        #     if (math.hypot(dyn.x - env.current_state.x, dyn.y - env.current_state.y) < 5 or\
        #                 math.hypot(dyn.x - env.current_state.x, dyn.y - env.current_state.y) < 5):
        #         env.render(100, save_image=False)
        #         print("Show the render")
        
        delta_distance = math.hypot(env.current_state.x - env.goal.x, env.current_state.y - env.goal.y) 
        if (delta_distance < 1.0):
            if not goal:
                isDone = True
                break
            else:
                if (delta_distance < 0.5):
                    delta_orientation = abs(normalizeAngle(env.goal.theta - env.current_state.theta))
                    if delta_orientation < (math.pi / 12.):
                        isDone = True
                        break
        if agent.config["model"]["use_lstm"]:
            action, state, logits = agent.compute_single_action(observation, state=state, prev_action=prev_action)
            prev_action = action
        else:
            start_time = time.time()
            action = agent.compute_single_action(observation)
            end_time = time.time()
            steering_time += (end_time - start_time)
        
        observation, reward, isDone, info = env.step(action, next_dyn_states=dyn_obstacles)
        if "EuclideanDistance" in info:
            if min_distance >= info["EuclideanDistance"]:
                min_distance = info["EuclideanDistance"]

        sum_reward += reward
        if save_image:
            images.append(env.render(sum_reward))
        else:
            images.append(env.current_state)
        t += 1

        if "SoftEps" in info or "Collision" in info:
            if "Collision" in info:
                collision = True
                # print("@@@ Collision @@@")
            isDone = False
            break

    # if not goal and not isDone:
    #     if "SoftEps" not in info and "Collision" not in info:
    #         final_distance = math.hypot(env.current_state.x - env.goal.x, env.current_state.y - env.goal.y)
    #         # print(f"advanced distance %: {(final_distance / initial_distance) * 100}")
    #         # print("The new option for dynamic obstacles")
    #         if (final_distance / initial_distance) <= 0.5:
    #             isDone = True
        
    if save_image:
        images = np.transpose(np.array(images), axes=[0, 3, 1, 2])

    steering_time += env.collision_time

    return isDone, images, min_distance, steering_time

def steering_DWA(env, agent, max_steps=150, idx=None, save_image=False, val_key=None, goal=False, dyn_trajectories=[]):
    observation = env.reset(idx=idx, fromTrain=False, val_key=val_key, rrt=True)
    dyn_obs_trajectories = list(dyn_trajectories)
    env.n_beams = 9
    env.MAX_DIST_LIDAR = 5
    env.view_angle = math.pi / 3.
    env.frame_stack = 1
    id_dyn_obst = 0
    images = []
    sum_reward = 0
    if save_image:
        images.append(env.render(sum_reward))
    else:
        images.append(env.current_state)
    isDone = False
    min_distance = 0
    steering_time = 0
    old_distance = float('inf')

    for _ in range(max_steps):
        # env.render(100, save_image=False)
        id_dyn_obst += 1
        dyn_obstacles = []
        for dyn_obst in dyn_obs_trajectories:
            if id_dyn_obst < len(dyn_obst):
                dyn_obstacles.append(dyn_obst[id_dyn_obst])
            else:
                dyn_obstacles.append(dyn_obst[-1])

        # start_time = time.time()
        best_actions = planningDWA(env, dyn_obstacles)
        # end_time = time.time()
        # steering_time += (end_time - start_time)
        # print(f"best_actions {best_actions}")
        observation, reward, isDone, info = env.step(best_actions, next_dyn_states=dyn_obstacles)
        sum_reward += reward
        if save_image:
            images.append(env.render(sum_reward))
        else:
            images.append(env.current_state)
        delta_distance = math.hypot(env.current_state.x - env.goal.x, env.current_state.y - env.goal.y) 
        # print(f"delta_distance: {delta_distance}")
        # if old_distance > delta_distance:
        #     old_distance = delta_distance
        # else:
        #     print(f"delta_distance: {delta_distance}")
        #     isDone = True
        #     break
        if "Collision" in info:
            isDone = False
            break
        
        if not goal:
            if delta_distance <= 3.0 or isDone:
                isDone = True
                break
        else:
            if delta_distance <= 1.0 or isDone:
                isDone = True
                break

    steering_time = env.collision_time

    return isDone, images, min_distance, steering_time

def getTrajectory(env, agent, valTask,  obstacle_map=[], dyn_trajectories=[], saveImage=False, goal=False, dwa=False):
    
    from_node = deepcopy(valTask[0][0])
    new_node = deepcopy(valTask[0][1])
    sx, sy, stheta, sv, sst = from_node
    gx, gy, gtheta, gv, gst = new_node
    
    transform = Transformation()

    _, _ = transform.rotate([sx, sy, stheta], [gx, gy, gtheta])
    # start_transform.append(sv)
    # goal_transform.append(gv)
    # start_transform.append(sst)
    # goal_transform.append(gst)
    dyn_obstacles = []
    for dyn_obst in dyn_trajectories:
        dyn_obstacles.append(dyn_obst[0])
    valTasks = [(from_node, new_node, dyn_obstacles)]
    # print(f"from_node {from_node}")
    # print(f"new_node {new_node}")

    env.valTasks = {}
    env.valTasks["map0"] = valTasks
    env.maps_init = {}
    env.maps_init["map0"] = obstacle_map
    # env.dynamic_obstacles = dyn_obstacles
    # print(f"dyn_obstacles {dyn_obstacles}")
    # print(obstacle_map)
    env.maps = deepcopy(env.maps)

    # print(f"env.valTasks :{env.valTasks}")
    if not dwa:
        isDone, images, _, steering_time = validate_task(env, agent, val_key="map0", goal=goal, dyn_trajectories=dyn_trajectories)
    else:
        isDone, images, _, steering_time = steering_DWA(env, agent, val_key="map0", goal=goal, dyn_trajectories=dyn_trajectories)
    
    lst_new_params = []
    for i in range(len(images)):
        curr_state = images[i]
        x1 = curr_state.x
        y1 = curr_state.y
        theta1 = curr_state.theta
        v1 = curr_state.v
        st1 = curr_state.steer
        # x1, y1, theta1, v1, st1
        x1, y1, theta1 = transform.inverseRotate([x1, y1, theta1])
        lst_new_params.append((x1, y1, theta1, v1, st1))
        # print(lst_new_params)
    return isDone, lst_new_params if isDone else [], steering_time


# def validate(agent, valTask, obstacle_map, saveImage=False, goal=False):
#     max_time_steps = train_config["max_time_steps"]
#     frame_stack = train_config["frame_stack"]
#     frame_skip = train_config["frame_skip"]
#     env.valTasks = valTask
#     env.obstacle_map_init = obstacle_map
#     env.obstacle_map = deepcopy(obstacle_map)
#     observation = env.reset(fromTrain=False)
#     lst_params = []
#     lst_params.append(env.current_state)
#     isDone = False
#     t = 0
#     lst_states = []
    
#     while not isDone and t < max_time_steps:

#         lst_states.append(observation)
#         if len(lst_states) > frame_stack:
#             lst_states = lst_states[1:]
#         else:
#             lst_states = lst_states * frame_stack
                    
#         if not t % frame_skip:
#             frame_state = np.hstack(lst_states)
#             action = agent.get_action(frame_state, deterministic=True)

#         observation, reward, isDone, info = env.step(action, soft=False)
#         # if (observation[19] ** 2 + observation[20] ** 2) ** 0.5 < 0.5  and abs(normalizeAngle(observation[21])) <= pi/36:
#         #     isDone = True
#         # if (observation[19] ** 2 + observation[20] ** 2) ** 0.5 < 0.5:
#         #     if abs(normalizeAngle(observation[21])) <= pi/18:
#         #         isDone = True
#         if (observation[19] ** 2 + observation[20] ** 2) ** 0.5 < 0.5:
#             if goal:
#                 if abs(normalizeAngle(observation[21])) <= pi/18:
#                     isDone = True
#             else:
#                 isDone = True
#         lst_params.append(env.current_state)                                                                                            
#         t += 1

#         if "SoftEps" in info or "Collision" in info:
#             break
        
#     env.close()

#     # return isDone, lst_params if isDone else []
#     return isDone, lst_params


# def getTrajectory(agent, valTask,  obstacle_map=[], saveImage=False, goal=False):
#     from_node = deepcopy(valTask[0][0])
#     new_node = deepcopy(valTask[0][1])
#     # drawBB(from_node, color="--g")
#     # drawBB(new_node, color="--b")
#     sx, sy, stheta, sv, sst = from_node
#     # print("start: ", sx, " ", sy)
#     gx, gy, gtheta, gv, gst = new_node
#     # print("goal: ", gx, " ", gy)
#     transform = Transformation()
#     start_transform, goal_transform = transform.rotate([sx, sy, stheta], [gx, gy, gtheta])
#     start_transform.append(sv)
#     goal_transform.append(gv)
#     start_transform.append(sst)
#     goal_transform.append(gst)
#     valTasks = [(start_transform, goal_transform)]
#     # print("okey")
#     for index in range(len(obstacle_map)):
#         # print("okey1")
#         obst = getBB([obstacle_map[index][0], obstacle_map[index][1], obstacle_map[index][4]], w=obstacle_map[index][3], l=obstacle_map[index][2], ego=False)
#         # print("okey2")
#         # plt.plot([obst[(i + 1) % len(obst)][0] for i in range(len(obst) + 1)], [obst[(i + 1) % len(obst)][1] for i in range(len(obst) + 1)], '--k')

#     new_obstacle_map = []
#     for index in range(len(obstacle_map)):
#         x, y, length, width, theta = obstacle_map[index]
#         state = transform.rotateState([x, y, theta])
#         new_obstacle_map.append([state[0], state[1], length, width, state[2]])
#         # print([state[0], state[1], length, width, state[2]])
#     #     obst = getBB([state[0], state[1], state[2]], w=width, l=length, ego=False)
#     #     plt.plot([obst[(i + 1) % len(obst)][0] for i in range(len(obst) + 1)], [obst[(i + 1) % len(obst)][1] for i in range(len(obst) + 1)], '-r')
#     # drawBB(start_transform, color="-g")
#     # drawBB(goal_transform, color="-b")
    
#     # new_obstacle_map = []
#     done, lst_params = validate(agent, valTasks, obstacle_map=new_obstacle_map, goal=goal)
    
#     print("done!!: ", done)

#     # if done:
#     #     print("len(lst_params)!!: ", len(lst_params))
#     #     for i in range(len(lst_params)):
#     #         drawBB(lst_params[i])
#     #     plt.grid(True)
#     #     plt.show()
        
#     lst_new_params = []
#     for i in range(len(lst_params)):
#         x1, y1, theta1, v1, st1 = lst_params[i]
#         x1, y1, theta1 = transform.inverseRotate(x1, y1, theta1)
#         lst_new_params.append((x1, y1, theta1, v1, st1))
    


#     return done, lst_new_params if done else []
    