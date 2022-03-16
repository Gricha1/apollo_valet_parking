import math
import numpy as np

pointCloud = []
possible_accelerations = [-5, 5]
possible_rotation_rates = [-math.pi / 12, math.pi / 12]

def calculateVelocityCost(new_state, goal_state, vehicle):
    # print(f"new_state.v: {new_state.v}")
    # print(f"goal_state.v: {goal_state.v}")
    # return 1.0
    # return abs(goal_state.v - new_state.v) / vehicle.max_vel
    return (vehicle.max_vel - new_state.v) / vehicle.max_vel

def calculateHeadingCost(goal_state, next_state):
    dx = goal_state.x - next_state.x
    dy = goal_state.y - next_state.y
    # angleError = math.atan2(dy, dx)
    # angleCost = angleError - next_state.theta
    # print(math.fabs(math.atan2(math.sin(angleCost), math.cos(angleCost))))
    # return math.fabs(math.atan2(math.sin(angleCost), math.cos(angleCost)))
    return math.hypot(dx, dy)

def calculateClearanceCost(beams, max_beam, info):
    # min_radius = float('inf')
    if "Collision" in info:
        return float('inf')
    mean = np.mean(np.array(beams))
    # for i in range(len(beams)):
    #     if beams[i] < min_radius:
    #         min_radius = beams[i]
    # print(1.0 / min_radius)
    
    return max_beam / mean
    
def planningDWA(env, dyn_obstacles):
    # print(f"env.MAX_DIST_LIDAR {env.MAX_DIST_LIDAR}")
    # print(f"env.n_beams {env.n_beams}")
    init_state = env.current_state
    goal_state = env.goal
    vehicle = env.vehicle 
    min_cost = float('inf')
    cost = 0
    best_actions = [0., 0.]
    # velocity_koeff = 0
    # heading_koeff = 0
    # clearance_koeff = 0
    max_acc = -5
    max_rotation = math.pi / 12.
    possible_accelerations = list(np.linspace(-max_acc, max_acc, 5))
    possible_rotation_rates = list(np.linspace(-max_rotation, max_rotation, 7))
    heading_koeff = 0.15
    clearance_koeff = 1.0
    velocity_koeff = 1.0
    for acc in possible_accelerations:
        for ang_vel in possible_rotation_rates:
            env.current_state = init_state
            actions = []
            actions.append(acc)
            actions.append(ang_vel)
            # pPose = motion(pPose, pVelocity, config.predictTime);
            # print(f"x: {current_state.x}")
            # print(f"y: {current_state.y}")
            observation, reward, isDone, info = env.step(actions, next_dyn_states=dyn_obstacles)
            # new_state, _, _ = vehicle.dynamic(current_state, actions)
            new_state = env.current_state
            # print(observation)
            beams = observation[:9]
            # print(len(beams))
            # print(f"newx: {new_state.x}")
            # print(f"newy: {new_state.y}")
            # beams = []
            # if len(env.obstacle_segments) > 0 or len(env.dyn_obstacle_segments) > 0:
            #     for angle in np.linspace(-env.view_angle, env.view_angle, env.n_beams):
            #         beam = env.sendBeam(new_state, angle)
            #         beams.append(beam)
            # else:
            #     for angle in np.linspace(-env.view_angle, env.view_angle, env.n_beams):
            #         beams.append(env.MAX_DIST_LIDAR)
            cost = velocity_koeff * calculateVelocityCost(new_state, goal_state, vehicle) +\
            heading_koeff * calculateHeadingCost(goal_state, new_state) +\
            clearance_koeff * calculateClearanceCost(beams, env.MAX_DIST_LIDAR, info)
            # print(f"cost {cost}")
            if (cost < min_cost):
                min_cost = cost
                best_actions = actions
                # if abs(actions[0]) < 0.001 \
                #         and abs(new_state.steer) < 0.001:

    env.current_state = init_state
    # print(f"best_actions {best_actions}")
    # print(f"min_cost {min_cost}")
    return best_actions