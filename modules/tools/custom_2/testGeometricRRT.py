from pickle import TRUE
from re import T
import time
from matplotlib.pyplot import figure
from planning.RRT import *
from validateModel import *
from planning.validate import *

print("start " + __file__)

RL = True
expand_dis = 10
show_animation = True
RANDOM = False

obstacles = [
            [50, 15, 0 * pi/2, 5, 30], 
            [50, 35, 0 * pi/2, 5, 40],
            [55, 55, 0 * pi/2, 5, 25],
            ]

lst_starts = [
            [15, 15, degToRad(90), kmToM(0), degToRad(0)],
            # [20, 55, degToRad(-90), kmToM(0), degToRad(0)],
            # [5, 35, degToRad(90), kmToM(0), degToRad(0)],
            # [5, 35, degToRad(-90), kmToM(0), degToRad(0)],
            # [40, 45, degToRad(0), kmToM(0), degToRad(0)],
            # [40, 45, degToRad(180), kmToM(0), degToRad(0)],
            # [40, 5, degToRad(0), kmToM(0), degToRad(0)],
            # [40, 5, degToRad(180), kmToM(0), degToRad(0)],
            # [40, 25, degToRad(0), kmToM(0), degToRad(0)],
            # [40, 25, degToRad(180), kmToM(0), degToRad(0)]
]

goal = [90., 15., degToRad(-90), kmToM(0), degToRad(0)]



dt = 0.1
width = 100
height = 60

dyn_obstacles = []
total_tasks = 2

for index in range(10):
    rrt = RRTGeometric(
            start=lst_starts[0],
            goal=goal,
            agent = agent,
            env = env,
            obstacles=obstacles,
            width=width,
            height=height,
            dyn_obstacles=dyn_obstacles,
            rl=RL,
            expand_dis=expand_dis,
            animation=show_animation,
            random=RANDOM,
            smoothing=True)

    start_time = time.time()
    path, pathr = rrt.planning()
    end_time = time.time()
    print("time: ", end_time - start_time)
    print(f"path {path}")
    print(f"pathr {pathr}")
    if True:
        plt.clf()
        rrt.draw_graph()
        plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
        for i in range(1, len(path)):
            plt.plot([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], "-b", linewidth=line_size)  
        plt.grid(True)
        plt.pause(1.0)
        # plt.close('all')
        plt.show()

    rl_trajectory = []
    stheta = lst_starts[0][2]
    sv = lst_starts[0][3]
    ssteer = lst_starts[0][4]
    gv = 5
    gsteer = 0
    sx = path[len(path)-1][0]
    sy = path[len(path)-1][1]

    for i in range(len(path)-1, 0, -1):
        print("sx and sy: ", sx, " ", sy)
        gx = path[i-1][0]
        gy = path[i-1][1]
        print("gx and gy: ", gx, " ", gy)
        start_transform = [sx, sy, stheta, sv, ssteer]
        gtheta = math.atan2(gy - sy, gx - sx)
        if i > 2:
            gtheta += math.atan2(path[i-2][1] - gy, path[i-2][0] - gx)
            gtheta /= 2.
        goal_transform = [gx, gy, gtheta, gv, gsteer]
        valTasks = [(start_transform, goal_transform)]
        done, lst_params, _ = getTrajectory(env, agent, valTasks,  obstacle_map=deepcopy(obstacles))
        if len(lst_params) > 0:
            sx = lst_params[-1][0]
            sy = lst_params[-1][1]
            stheta = lst_params[-1][2]
            sv = lst_params[-1][3]
            ssteer = lst_params[-1][4]
            rl_trajectory.extend(lst_params)
        else:
            break
    if path is not None:
        plt.clf()
        rrt.draw_graph()
        plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
        for i in range(1, len(path)):
            plt.plot([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], "-k", linewidth=line_size)  
        for state in rl_trajectory:
            drawBB(state, draw_arrow=False)
        plt.grid(True)
        plt.pause(1.0)
        plt.savefig('results/GeometriPath' + str(RL) + str(index) + '.png')
        # plt.close('all')
        plt.show()