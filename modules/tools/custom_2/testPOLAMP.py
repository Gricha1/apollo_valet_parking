from pickle import TRUE
from re import T
import time
import os
from matplotlib.animation import FuncAnimation
from matplotlib.pyplot import figure
from planning.RRTRLDYNOBS import *
from validateModel import agent, env, vehicle_config, curriculum_name
from EnvLib.ObstGeomEnv import *
from planning.generateMap import saveDynamicTrajectories, getTaskAndDynamicTrajectories, readTasks
print("start " + __file__)

def generateDynamicTrajectories(task, num_dyn_obst=5, steps=1000):
    start, goal = task
    dyn_obstacles = []
    for _ in range(num_dyn_obst):
        state = []
        min_box_x = min(start[0], goal[0])
        max_box_x = max(start[0], goal[0])
        min_box_y = min(start[1], goal[1])
        max_box_y = max(start[1], goal[1])
        dx = np.random.randint(min_box_x - 5 , max_box_x + 5)
        dy = np.random.randint(min_box_y - 5, max_box_y + 5)
        while (hypot(dx - start[0], dy - start[1]) < 8): 
            dx = np.random.randint(min_box_x - 5 , max_box_x + 5)
            dy = np.random.randint(min_box_y - 5, max_box_y + 5)
        state.append(dx)
        state.append(dy)
        state.append(degToRad(np.random.randint(0, 4) * 90))
        state.append(kmToM(np.random.randint(0, 10)))
        state.append(0)
        dyn_obstacles.append(state)

    dyn_trajectories = []
    for dyn_obst in dyn_obstacles:
        dyn_trajectory = []
        dyn_state = State(dyn_obst[0], dyn_obst[1], dyn_obst[2], dyn_obst[3], dyn_obst[4])
        random_time_step = 1
        for i in range(steps): 
            if (not i % random_time_step):
                # random_time_step = 10 + np.random.randint(-3, 3 + 1)
                random_time_step = int(np.random.normal(10, 1))
                dyn_acc = np.random.randint(-vehicle_config.max_acc, vehicle_config.max_acc + 1)
                # print(radToDeg(vehicle_config.max_ang_vel))
                dyn_ang_vel = np.random.normal(0, 2) * 5
                # print(f"dyn_acc: {dyn_acc}")
                # print(f"dyn_ang_vel: {dyn_ang_vel}")
            dyn_state, _, _ = vehicle_config.dynamic(dyn_state, [dyn_acc, degToRad(dyn_ang_vel)])
            if dyn_state.x < 0 or dyn_state.x > width or dyn_state.y < 0 or dyn_state.y > height:
                dyn_state.x = np.random.randint(2, width - 2) 
                dyn_state.y = np.random.randint(2, height - 2)
                dyn_state.theta = degToRad(np.random.randint(0, 4) * 90)
            new_state = [dyn_state.x, dyn_state.y, dyn_state.theta, dyn_state.v, dyn_state.steer]
            # print(new_state)
            dyn_trajectory.append(new_state)
        dyn_trajectories.append(dyn_trajectory)
    return dyn_trajectories

def update(i):
    plt.clf()
    x_min = rrt.trajectory[0][i] - x_delta
    x_max = rrt.trajectory[0][i] + x_delta
    plt.xlim(x_min, x_max)
    y_min = rrt.trajectory[1][i] - y_delta
    y_max = rrt.trajectory[1][i] + y_delta
    plt.ylim(y_min, y_max)
    rrt.draw_obstacles(show_dyn_obst=False)
    plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
    drawBB([rrt.trajectory[0][i], rrt.trajectory[1][i], rrt.trajectory[2][i]], color="green", color_arrow='red')

    for t in range(len(lst_dyn_trajectories[id])):
        if i < len(lst_dyn_trajectories[id][t]):
            drawBB(lst_dyn_trajectories[id][t][i], color="-m")
        else:
            drawBB(lst_dyn_trajectories[id][t][-1], color="-m")
    

obstacles_map0 = [
            [15, 30, 0 * pi/2, 20, 5], 
            [50, 15, 0 * pi/2, 5, 10],
            [50, 35, 0 * pi/2, 5, 10],
            [50, 55, 0 * pi/2, 5, 10],
            [85, 30, 0 * pi/2, 20, 5],
            ]

obstacles_map1 = [
            [25, 45, 0 * pi/2, 5, 15],
            [25, 25, 0 * pi/2, 5, 15],
            [25, 5, 0 * pi/2, 5, 15],
            [70, 55, 0 * pi/2, 5, 20],
            [70, 35, 0 * pi/2, 5, 20],
            [70, 15, 0 * pi/2, 5, 20],
            ]

obstacles_map2 = [
            [50, 15, 0 * pi/2, 5, 30], 
            [25, 35, 0 * pi/2, 5, 15],
            [75, 35, 0 * pi/2, 5, 15],
            [50, 55, 0 * pi/2, 5, 30],
            ]

env_maps = {}
env_maps["obstacles_map0"] = obstacles_map0
env_maps["obstacles_map1"] = obstacles_map1
env_maps["obstacles_map2"] = obstacles_map2

# for index in range(2):
# lst_tasks_1 = readTasks("maps/random1_tasks.txt")
# map0_task1 = ([5., 5., 0., 0., 0.], [90., 55., 0., 0., 0.])
# map0_task2 = ([5., 55., 0., 0., 0.], [90., 5., 0., 0., 0.])
# map1_task1 = ([5., 5., math.pi / 2., 0., 0.], [95., 55., math.pi / 2., 0., 0.])
# map1_task2 = ([5., 55., 0., 0., 0.], [90., 5., 0., 0., 0.])
# map2_task1 = ([5., 5., math.pi / 2., 0., 0.], [95., 55., 0., 0., 0.])
# map2_task2 = ([5., 55., 0., 0., 0.], [95., 15., 0., 0., 0.])
# task = map2_task2


### Tasks with dynamic obstacles
dyn_env = False
maps = {}
if dyn_env:
    name_map = "map0"
    num_tasks = 4
    lst_dyn_trajectories = []
    lst_task_dyn = []
    step = 10
    min_count = 0
    max_count = 71
    file = "maps/dynamic_experiments_" + name_map + "/task1/"
    for id in range(num_tasks):
        total_dyn_trajectories = []
        for num_dyn_obst in range(min_count, max_count, step):
            # task = lst_tasks_1[id_rand]
            # Genrating tasks from scratch
            # if num_dyn_obst > 0:
            #     dyn_trajectories = generateDynamicTrajectories(task, num_dyn_obst=step)
            #     total_dyn_trajectories.extend(dyn_trajectories)
            #     print(f"len(total_dyn_trajectories) : {len(total_dyn_trajectories)}")
            # saveDynamicTrajectories(file + "final_task", task, total_dyn_trajectories, id, num_dyn_obst)
            task, new_dyn_trajectories = getTaskAndDynamicTrajectories(file + "final_task" + str(id) + "_" + str(num_dyn_obst) + ".txt")
            lst_dyn_trajectories.append(new_dyn_trajectories)
            lst_task_dyn.append(task)
    obstacles = env_maps["obstacles_" + name_map]
    maps[name_map + "_dyn_obst"] = (obstacles, lst_task_dyn, lst_dyn_trajectories)
    frequency = max_count // step + 1
### Tasks in static environments
else:
    name_map = "map1"
    file_map = "maps/static_experiments_" + name_map + "/"
    lst_tasks = readTasks(file_map + "tasks.txt")
    obstacles = env_maps["obstacles_" + name_map]
    maps[name_map + "_stat_obst"] = (obstacles, lst_tasks, [[] for i in range(len(lst_tasks))])
    for t, tasks in enumerate(lst_tasks):
        start, goal = tasks
        plt.plot(start[0], start[1], "rH")
        drawBB(start, color="-r")
        plt.annotate(str(t), (start[0], start[1]), (start[0]+1, start[1]+1))
        plt.plot(goal[0], goal[1], "cH")
        plt.annotate(str(t), (goal[0], goal[1]), (goal[0]+1, goal[1]+1))
        drawBB(goal, color="-c")
        plt.plot([start[0], goal[0]], [start[1], goal[1]], '--g')
        for new_bb in obstacles:
            drawBB(new_bb, color="-k", ego=False, draw_arrow=False)
    plt.savefig(file_map + 'image_tasks.png')
    # plt.show()

RL = True
dwa = False
expand_dis = 10
show_animation = False
RANDOM = False
width = 100
height = 60
radius = 40
dt = 0.1
if curriculum_name == "rllib_ppoWithEuclidean":
    add_name = "eu_"
elif curriculum_name == "rllib_ddpgEuclidean":
    add_name = "ddpg_"
elif curriculum_name == "rllib_ppoWithOrientation":
    add_name = ""
elif curriculum_name == "rllib_ppoStaticObstacles": 
    add_name = "rl_stat_"
else:
    add_name = "Error"

if dyn_env:
    max_iter = 1500
    main_file = "results/rl_dynamic_" + add_name + name_map + "/"
    # print(f"main_file {main_file}")
    if dwa:
        max_iter = 2000
        # radius = 20
        main_file = "results/dwa_dynamic_" + name_map + "/"
    print(f"main_file {main_file}")
else:
    if RL:
        # add_name =
        max_iter = 1500
        main_file = "results/rl_static_" + add_name + name_map + "/"
        # print(f"main_file {main_file}")
        if dwa:
            radius = 20
            main_file = "results/dwa_static_" + name_map + "/"
        print(f"main_file {main_file}")
    else:
        # expand_dis = 5
        max_iter = 3000
        main_file = "results/posq_static_" + name_map + "/"

if not os.path.exists(main_file):
    os.makedirs(main_file)

total_tasks = 10
for key in maps:
    obstacles, lst_tasks, lst_dyn_trajectories = maps[key]
    # Weight of euclidean agent
    key_mod = key
    # key_mod = key + "eucl_dyn_obst" 
    # for id in range(len(lst_tasks) - 1, -1, -1):
    for id in range(7, 10):
        print(f"@@@@@@ {id}")
        total_success = 0
        avg_time_to_reach = 0
        avg_samples = 0
        avg_success_samples = 0
        algorithm_time = 0
        lst_samples = []
        lst_success_samples = []
        lst_time_to_reach = []
        lst_algorithm_time = []
        # print(f"len(lst_dyn_trajectories[id]) {len(lst_dyn_trajectories[id])}")
        # if (id % frequency) == 0 and id > 0:
        #     print("### Continue ###")
        #     continue
        show_video = False
        for k in range(total_tasks):
            print(f"###### {k}")
            rrt = RRT(
                    # start=lst_tasks[id][0],
                    # goal=lst_tasks[id][1],
                    start=[5, 5, math.pi / 2., 0., 0.],
                    goal=[95, 55, math.pi / 2., 0., 0.],
                    agent = agent,
                    env = env,
                    obstacles=obstacles,
                    width=width,
                    height=height,
                    dyn_trajectories=lst_dyn_trajectories[id],
                    rl=RL,
                    dwa=dwa,
                    expand_dis=expand_dis,
                    radius=radius,
                    animation=show_animation,
                    random=RANDOM,
                    max_iter=max_iter)

            
            start_time = time.time()
            path, pathr = rrt.planning()
            end_time = time.time()
            print("time: ", end_time - start_time)

            samples = rrt.number_samples
            success_samples = rrt.number_success_samples
            avg_samples += samples
            avg_success_samples += success_samples
            lst_samples.append(samples)
            lst_success_samples.append(success_samples)
            
            if path is not None:
                print("Path was found")
                total_success += 1
                time_to_reach = len(rrt.trajectory[0])
                lst_time_to_reach.append(time_to_reach)
                avg_time_to_reach += time_to_reach
                time_rrt = end_time - start_time - rrt.simulation + rrt.steering_time
                lst_algorithm_time.append(time_rrt)
                algorithm_time += time_rrt
            else:
                lst_time_to_reach.append(0)
                lst_algorithm_time.append(0)
                print("Path was not found")
            if not show_video and path is not None:
                show_video = True
                plt.clf()
                figsize=(10, 8)
                fig, ax = plt.subplots(figsize=figsize)
                x_delta = 20
                y_delta = 20
                x_min = rrt.trajectory[0][0] - x_delta
                x_max = rrt.trajectory[0][0] + x_delta
                # plt.xlim(x_min, x_max)
                y_min = rrt.trajectory[1][0] - y_delta
                y_max = rrt.trajectory[1][0] + y_delta
                # plt.ylim(y_min, y_max)
                # rrt.draw_graph()
                # rrt.draw_obstacles()
                plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
                # anim = FuncAnimation(fig, update, frames=np.arange(0, len(rrt.trajectory[0])), interval=200)
                # if dyn_env:
                #     addit_name = str(key_mod) + str(id // frequency) + "_" + str(len(lst_dyn_trajectories[id]))
                # else:
                #     addit_name = str(key_mod) + str(id)
                # anim.save(main_file + 'video_task'+ addit_name + '.mp4')
                for i in range(len(rrt.trajectory[0])):
                    # plt.clf()
                    # plt.gcf().canvas.mpl_connect(
                    # 'key_release_event',
                    # lambda event: [exit(0) if event.key == 'escape' else None])
                    # rrt.draw_obstacles(show_dyn_obst=False)
                    # plt.plot(rrt.trajectory[0], rrt.trajectory[1], '-r', linewidth=line_size)
                    drawBB([rrt.trajectory[0][i], rrt.trajectory[1][i], rrt.trajectory[2][i]], draw_arrow=False)

                    # for t in range(len(lst_dyn_trajectories[id])):
                    #     if i < len(lst_dyn_trajectories[id][t]):
                    #         drawBB(lst_dyn_trajectories[id][t][i], color="-m")
                    #     else:
                    #         drawBB(lst_dyn_trajectories[id][t][-1], color="-m")
                    
                    # x_min = rrt.trajectory[0][i] - x_delta
                    # x_max = rrt.trajectory[0][i] + x_delta
                    # ax.set_xlim(x_min, x_max)
                    # y_min = rrt.trajectory[1][i] - y_delta
                    # y_max = rrt.trajectory[1][i] + y_delta
                    # ax.set_ylim(y_min, y_max)
                    # plt.pause(0.1)    
                    # plt.close('all')
                rrt.draw_obstacles()
                plt.grid(True)
                plt.pause(0.1)
                plt.savefig('results/figure' + str(key_mod) + str(RL)+ str(0) + "_" + str(len(lst_dyn_trajectories[id])) + '.png')
                plt.close('all')
                plt.show()

        if total_success < 1:
            total_success = 1

        if dyn_env:
            index = id // frequency
        else:
            index = id
        additional = "_{}_{}_{}_{}.{}".format(key_mod, RL, index, len(lst_dyn_trajectories[id]), "txt") 
        with open("{}{}{}".format(main_file, "metricsRRT", additional), "w") as output:
            output.write("succes rate: " + str(total_success / total_tasks) + "\n")
            output.write("total samples: " + str(avg_samples / total_tasks) + "\n")
            output.write("successed samples: " + str(avg_success_samples / total_tasks) + "\n")
            output.write("average time to reach: " + str(avg_time_to_reach / total_success) + "\n")
            output.write("average algorithm time: " + str(algorithm_time / total_success) + "\n")
        
        with open("{}{}{}".format(main_file, "TimeToReach", additional), 'w') as output:
            for t in lst_time_to_reach:
                output.write(str(t) + '\n')
        
        with open("{}{}{}".format(main_file, "AlgorithmTime", additional), 'w') as output:
            for t in lst_algorithm_time:
                output.write(str(t) + '\n')

        with open("{}{}{}".format(main_file, "Samples", additional), 'w') as output:
            for t in lst_samples:
                output.write(str(t) + '\n')
        
        with open("{}{}{}".format(main_file, "SuccessedSamples", additional), 'w') as output:
            for t in lst_success_samples:
                output.write(str(t) + '\n')
