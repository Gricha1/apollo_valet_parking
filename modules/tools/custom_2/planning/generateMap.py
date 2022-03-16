from typing import Counter
from modules.tools.custom_2.planning.collision import *
from modules.tools.custom_2.planning.utilsPlanning import*
import numpy as np
import random
from pickle import TRUE
from re import T
import time
from matplotlib.pyplot import figure
from modules.tools.custom_2.planning.RRTRLDYNOBS import *
from modules.tools.custom_2.planning.collision import *

def cutParkingPart(lst_bb, width=40, length=40):
    new_lst_bb = []
    map_x1 = np.random.randint(0, 6) * 10
    map_x2 = map_x1 + length
    map_y1 = np.random.randint(0, 2) * 10
    map_y2 = map_y1 + width

    # map_x1 = 0
    # map_x2 = 40
    # map_y1 = 15
    # map_y2 = 55

    obstacle_map = [(map_x1 + map_x2) / 2., (map_y1+ map_y2) / 2., 0 * pi/2, 20, 20]

    for bb in lst_bb:
        bb_x1 = bb[0][0]
        bb_x2 = bb[1][0]
        bb_y1 = bb[0][1]
        bb_y2 = bb[3][1]
        flag_x = (map_x1 < bb_x1 and bb_x1 < map_x2) or (map_x1 < bb_x2 and bb_x2 < map_x2)
        flag_y = (map_y1 < bb_y1 and bb_y1 < map_y2) or (map_y1 < bb_y2 and bb_y2 < map_y2) 
        if(flag_x or flag_y):
            if (flag_x and flag_y):
                new_bb_x1 = max(bb_x1, map_x1)
                new_bb_x2 = min(bb_x2, map_x2)
                new_bb_y1 = max(bb_y1, map_y1)
                new_bb_y2 = min(bb_y2, map_y2)
                # new_lst_bb.append([(new_bb_x1, new_bb_y1), (new_bb_x2, new_bb_y1), (new_bb_x2, new_bb_y2), (new_bb_x1, new_bb_y2)])
                new_lst_bb.append([(new_bb_x1 + new_bb_x2) / 2., (new_bb_y1 + new_bb_y2) / 2., 0, (new_bb_y2 - new_bb_y1) / 2., (new_bb_x2 - new_bb_x1) / 2.])
            elif flag_x:
                if (bb_y1 <= map_y1 and map_y1 <= bb_y2) and (bb_y1 <= map_y2 and map_y2 <= bb_y2):
                    new_bb_x1 = max(bb_x1, map_x1)
                    new_bb_x2 = min(bb_x2, map_x2)
                    new_bb_y1 = map_y1
                    new_bb_y2 = map_y2
                    # new_lst_bb.append([(new_bb_x1, new_bb_y1), (new_bb_x2, new_bb_y1), (new_bb_x2, new_bb_y2), (new_bb_x1, new_bb_y2)])
                    new_lst_bb.append([(new_bb_x1 + new_bb_x2) / 2., (new_bb_y1 + new_bb_y2) / 2., 0, (new_bb_y2 - new_bb_y1) / 2., (new_bb_x2 - new_bb_x1) / 2.])
            elif flag_y:
                if (bb_x1 <= map_x1 and map_x1 <= bb_x2) and (bb_x1 <= map_x2 and map_x2 <= bb_x2):
                    new_bb_y1 = max(bb_y1, map_y1)
                    new_bb_y2 = min(bb_y2, map_y2)
                    new_bb_x1 = map_x1
                    new_bb_x2 = map_x2
                    # new_lst_bb.append([(new_bb_x1, new_bb_y1), (new_bb_x2, new_bb_y1), (new_bb_x2, new_bb_y2), (new_bb_x1, new_bb_y2)])
                    new_lst_bb.append([(new_bb_x1 + new_bb_x2) / 2., (new_bb_y1 + new_bb_y2) / 2., 0, (new_bb_y2 - new_bb_y1) / 2., (new_bb_x2 - new_bb_x1) / 2.])

    obstacle_map[0] = obstacle_map[0] - map_x1
    obstacle_map[1] = obstacle_map[1] - map_y1
    obstacle_map[2] = 0
    for bb in new_lst_bb:
        bb[0] = bb[0] - map_x1
        bb[1] = bb[1] - map_y1
    
    bb = getBB(obstacle_map, w=obstacle_map[3], l=obstacle_map[4], ego=False)
    # plt.plot([bb[(i + 1) % len(bb)][0] for i in range(len(bb) + 1)], [bb[(i + 1) % len(bb)][1] for i in range(len(bb) + 1)], '-b')
    
    return new_lst_bb

def collisionPoint(sx, sy, bb_obstacles):
    collision = False
    for obs in bb_obstacles:  
        if(intersectPoint([sx, sy], obs)):
            # print("intersectPoint")
            collision = True
            break   
        
    return collision

def collisionObstacles(start, goal, bb_obstacles):
    col = False
    sx = start[0]
    sy = start[1]
    stheta = start[2]
    gx = goal[0]
    gy = goal[1]
    gtheta = goal[2]
    width = 100
    height = 60
    if sx < 0 or sx > width or sy < 0 or sy > height:
        return True
    if gx < 0 or gx > width or gy < 0 or gy > height:
        return True

    bb1 = getBB([sx, sy, stheta])
    bb2 = getBB([gx, gy, gtheta])
    bb3 = getBB([sx, sy, stheta + math.pi / 2.])
    bb4 = getBB([gx, gy, gtheta + math.pi / 2.])

    for obs in bb_obstacles:
        if(intersect(bb1, obs)):
            # print("intersect")
            col = True
            break
        if(intersect(bb2, obs)):
            # print("intersect")
            col = True
            break
        if(intersect(bb3, obs)):
            # print("intersect")
            col = True
            break
        if(intersect(bb4, obs)):
            # print("intersect")
            col = True
            break

        # obs_x1 = obs[0][0]
        # obs_x2 = obs[1][0]
        # obs_y1 = obs[0][1]
        # obs_y2 = obs[3][1]

        # if obs_x1 == 0 and obs_x2 == 40:
        #     if (sy >= obs_y2 and gy <= obs_y1) or (sy <= obs_y1 and gy >= obs_y2):
        #         col = True
        #         break

        # if obs_y1 == 0 and obs_y2 == 40:
        #     if (sx >= obs_x2 and gx <= obs_x1) or (sx <= obs_x1 and gx >= obs_x2):
        #         col = True
        #         break

    return col

def generateTasks(obstacles, agent, env, number_of_tasks=1, RL = True, with_constraints=False, with_rrt=False):
    expand_dis = 10
    show_animation = True
    RANDOM = False
    tasks = []
    lst_starts = []
    lst_goals = []
    total_task = 0
    width = 100
    height = 60
    dyn_trajectories = []
    
    bb_obs = []
    for new_bb in obstacles:
        obs = getBB(new_bb, ego=False)
        bb_obs.append(obs)

    while (total_task != number_of_tasks):
        counter = 0
        sx = np.random.randint(3, width - 3 + 1)
        sy = np.random.randint(3, height - 3 + 1)
        if collisionPoint(sx, sy, bb_obs):
            continue

        gx = np.random.randint(3, width - 3 + 1)
        gy = np.random.randint(3, height - 3 + 1)

        while ((collisionPoint(gx, gy, bb_obs) or math.hypot(gx - sx, gy - sy) < 60)):
            gx = np.random.randint(3, width - 3 + 1)
            gy = np.random.randint(3, height - 3 + 1)
            counter += 1
            if counter > 10:
                break
        if counter > 10:
            continue

        if with_constraints:
            stheta = math.atan2(gy - sy, gx - sx) + degToRad(np.random.randint(-10, 10 + 1) * 5)
            gtheta = math.atan2(gy - sy, gx - sx) + degToRad(np.random.randint(-10, 10 + 1) * 5)
            stheta = normalizeAngle(stheta, -pi)
            gtheta = normalizeAngle(gtheta, -pi)
        else:
            stheta = normalizeAngle(np.random.randint(0, 3 + 1) * math.pi / 2., -pi)
            gtheta = normalizeAngle(np.random.randint(0, 3 + 1) * math.pi / 2., -pi)

        start = [sx, sy, stheta, 0., 0.]
        goal = [gx, gy, gtheta, 0., 0.]
        col = collisionObstacles(start, goal, bb_obs)
        if col:
            continue
        threshold = 6
        start1 = [sx + threshold * cos(stheta), sy + threshold * sin(stheta), stheta, 0., 0.]
        goal1 = [gx - threshold * cos(gtheta), gy - threshold * sin(gtheta), gtheta, 0., 0.]
        col = collisionObstacles(start1, goal1, bb_obs)
        if col:
            continue
        goal2 = [gx + threshold * cos(gtheta), gy + threshold * sin(gtheta), gtheta, 0., 0.]
        col = collisionObstacles(start1, goal2, bb_obs)
        if col:
            continue
        
        if with_rrt:
            RL = False
            rrt = RRT(
                start=start,
                goal=goal,
                agent = agent,
                env = env,
                obstacles=obstacles,
                width=width,
                height=height,
                dyn_trajectories=dyn_trajectories,
                rl=RL,
                expand_dis=expand_dis,
                animation=show_animation,
                random=RANDOM,
                max_iter=2000
                )

            path, pathr = rrt.planning()
            # print(pathr)
            if path is not None:
                print("Found solution")
                if with_constraints:
                    length = 0
                    aol = 0
                    for i in range(len(rrt.trajectory[0]) - 1):
                        dx = rrt.trajectory[0][i] - rrt.trajectory[0][i+1]
                        dy = rrt.trajectory[1][i] - rrt.trajectory[1][i+1]
                        length += hypot(dx, dy)
                        # dtheta = normalizeAngle(rrt.trajectory[2][i+1] - rrt.trajectory[2][i])
                        # aol += abs(dtheta)

                    for i in range(len(path) - 1):
                        dtheta = normalizeAngle(path[i + 1][2] - path[i][2])
                        aol += abs(dtheta)

                    aol = (aol / (len(path) - 1)) * 180 / pi

                    if length > 45 or aol > 80:
                        continue

                    if length <= 45:
                        lst_starts.append(start)
                        lst_goals.append(goal)
                        total_task += 1
                else:
                    lst_starts.append(start)
                    lst_goals.append(goal)
                    total_task += 1

                if  show_animation:
                    rrt.draw_graph()
                    rrt.draw_obstacles()
                    plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
                    for i in range(len(rrt.trajectory[0])):
                        drawBB([rrt.trajectory[0][i], rrt.trajectory[1][i], rrt.trajectory[2][i]], draw_arrow=False)
                    drawBB([rrt.trajectory[0][len(rrt.trajectory[0])-1], rrt.trajectory[1][len(rrt.trajectory[0])-1], rrt.trajectory[2][len(rrt.trajectory[0])-1]])
                    plt.grid(True)
                    plt.pause(0.1)
                    # plt.savefig('planning/figure.png')
                    # plt.show()
                    plt.close('all')

            else:
                print("Not found solution")
                None
        else:
            lst_starts.append(start)
            lst_goals.append(goal)
            total_task += 1

    for id in range(len(lst_goals)):
        tasks.append((lst_starts[id], lst_goals[id])) 

    return tasks

def readTasks(file):
    tasks = []
    with open(file, "r") as f:
        j = -1
        for line in f.readlines():
            if(j == -1):
                j += 1
                continue
            parameters = line.split('\t')
            # print(parameters)
            start = []
            goal = []
            for i in range(len(parameters) - 1):
                # print(parameters[i])
                if i > 4:
                    goal.append(float(parameters[i]))
                else:
                    start.append(float(parameters[i]))
            tasks.append((start, goal))
        #     plt.plot([start[0], goal[0]], [start[1], goal[1]], '-r')
        
        # plt.show()
        return tasks

def readDynamicTasks(file):
    tasks = []
    with open(file, "r") as f:
        j = -1
        for line in f.readlines():
            if(j == -1):
                j += 1
                continue
            parameters = line.split('\t')
            # print(parameters)
            start = []
            goal = []
            dynamic_obst = []
            for i in range(len(parameters) - 1):
                # print(parameters[i])
                if i < 5:
                    start.append(float(parameters[i]))
                elif i < 10:
                    goal.append(float(parameters[i]))
                else:
                    dynamic_obst.append(float(parameters[i]))
                # print(f"start {start}")
                # print(f"goal {goal}")
                # print(f"dynamic_obst {dynamic_obst}")
            tasks.append((start, goal, [dynamic_obst]))
        #     plt.plot([start[0], goal[0]], [start[1], goal[1]], '-r')
        
        # plt.show()
        return tasks

def readObstacleMap(file):
    obstacles = []
    with open(file, "r") as f:
        j = -1
        for line in f.readlines():
            if(j == -1):
                j += 1
                continue
            parameters = line.split('\t')
            obst = []
            for i in range(len(parameters) - 1):
                obst.append(float(parameters[i]))

            obstacles.append(obst)
        return obstacles

def getDynamicObstacles(start, goal, max_koef=4):
    slope = math.atan2(goal[1] - start[1], goal[0] - start[0])
    # length = math.hypot(goal[1] - start[1], goal[0] - start[0])
    # print(length)
    k1 = np.random.randint(1, max_koef + 1)
    # print(f"k1: {k1}")
    k2 = max_koef - k1
    orient = normalizeAngle(slope + math.pi / 2.) 
    medium = list((np.array(start) * k2 + np.array(goal) * k1) / max_koef)
    horizon = np.random.randint(2, 6)
    medium[0] += horizon * cos(orient)
    medium[1] += horizon * sin(orient)
    orient += degToRad(np.random.randint(-2, 2 + 1) * 5) + math.pi
    orient = normalizeAngle(orient)
    v = kmToM(np.random.randint(1, horizon + 1))
    st = degToRad(0.)
    medium[2] = orient
    medium[3] = v
    medium[4] = st

    return medium


def saveDynamicTrajectories(name, task, dyn_trajectories, index, num):
    with open(name + str(index) + "_" + str(num) + ".txt", 'w') as output:
        start, goal = task
        for param in start:
            output.write(str(param) + '\t')
        for param in goal:
            output.write(str(param) + '\t')
        output.write(str(len(dyn_trajectories)) + '\n')
        for trajectory in dyn_trajectories:
            for state in trajectory:
                for param in state:
                    output.write(str(round(param, 4)) + '\t')
                output.write('\n')
            output.write(str(-1) + '\n')

def getTaskAndDynamicTrajectories(file):
    task = []
    with open(file, "r") as f:
        j = 0
        start = []
        goal = []
        dynamic_trajectories = []
        trajectory = []
        for line in f.readlines():
            parameters = line.split('\t')
            if j == 0 :
                j += 1
                for i in range(len(parameters) - 1):
                    if i < 5:
                        start.append(float(parameters[i]))
                    else:
                        goal.append(float(parameters[i]))
                task = (start, goal)
            else:
                # print(len(parameters))
                if (len(parameters) == 1):
                    dynamic_trajectories.append(trajectory)
                    trajectory = []
                else:
                    state = []
                    for i in range(len(parameters) - 1):
                        state.append(float(parameters[i]))
                    trajectory.append(state)

        return task, dynamic_trajectories