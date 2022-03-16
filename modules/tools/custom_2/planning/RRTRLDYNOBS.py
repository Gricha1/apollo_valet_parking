import math
from pickle import TRUE
import random
from re import I
import matplotlib.pyplot as plt
import numpy as np
from modules.tools.custom_2.planning.validate import *
from modules.tools.custom_2.planning.posq import *
from modules.tools.custom_2.planning.collision import *
from modules.tools.custom_2.planning.utilsPlanning import *
import time

mark_size = 8
line_size = 2
NEAREST = 5

class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, theta, v, st):
            self.x = x
            self.y = y
            self.theta = theta
            self.v = v
            self.st = st
            self.x_r = x
            self.y_r = y
            self.theta_r = theta
            self.v_r = v
            self.st_r = st
            self.path_x = []
            self.path_y = []
            self.path_theta = []
            self.path_v = []
            self.path_st = []
            self.path_t = []
            self.g = 0
            self.parent = None
            self.time = 0.0
        
        def clear(self):
            self.path_x = []
            self.path_y = []
            self.path_theta = []
            self.path_v = []
            self.path_st = []
            self.path_t = []

    def __init__(self,
                 start,
                 goal,
                 agent,
                 env=None,
                 obstacles=[],
                 dyn_trajectories=[],
                 width=100,
                 height=40,
                 expand_dis=30.0,
                 goal_sample_rate=5,
                 max_iter=1500,
                 radius=40,
                 rl=True,
                 dwa=False,
                 animation=True,
                 random=False):
        """
        Setting Parameter

        start: Start State [x, y, theta, v]
        goal: Goal State [x, y, theta, v]
        map: Grid
        randArea: FromGrid?
        """
        self.dwa = dwa
        self.env = env
        self.start = self.Node(start[0], start[1], start[2], start[3], start[4])
        self.end = self.Node(goal[0], goal[1], goal[2], goal[3], goal[4])
        self.obstacles = obstacles
        # print(obstacles)
        self.radius = radius
        self.dyn_trajectories = dyn_trajectories
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.rl = rl
        self.node_list = []
        self.trajectory = []
        self.agent = agent
        self.rnd_delta_angle = 90
        self.rnd_min_velocity = 1
        self.rnd_max_velocity = 10
        self.time_steering = 0.
        self.number_samples = 0
        self.number_success_samples = 0
        self.animation = animation
        self.random = random
        self.frame = [[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]]

    def planning(self):
        """
        rrt path planning
        animation: flag for animation on or off
        """
        # plt.figure(figsize=(10, 10))
        if (self.check_collision(self.start, rrt=True) and self.check_collision(self.end, rrt=True)):
            print("The task is correctly")
        else:
            print("Error in the task")
            return False
        
        self.node_list = [self.start]
        self.steering_time = 0
        self.simulation = 0
        for i in range(self.max_iter):
            # print("$$ i: ", i)
            rnd_node = self.get_random_node()
            nearest_indexes = self.get_nearest_node_index(self.node_list, rnd_node)
            flag = False
            done = False
            self.number_samples += 1
            for n_index in nearest_indexes:
                # print("n_index: ", n_index)     
                nearest_node = self.node_list[n_index]
                flag, new_node = self.generateNewNode(nearest_node, rnd_node)
                if flag:
                    done, steering_time, sim_time = self.steer(nearest_node, new_node, self.agent)
                    self.steering_time += steering_time
                    self.simulation += sim_time
                if done:
                    break
            
            # if done:
            #     self.number_success_samples += 1
            #     d, theta = self.calc_distance_and_angle(new_node, self.end)
            #     if d < 0.5:
            #         done, steering_time, sim_time = self.steer(nearest_node, self.end, self.agent, goal=True)
            #         self.steering_time += steering_time
            #         self.simulation += sim_time
            #         if self.check_collision(self.end):
            #             if self.animation:
            #                 self.draw_graph(self.end)

            #             return self.generate_final_course(len(self.node_list) - 1)
            
            # if self.animation:
            #     self.draw_graph(rnd_node)

            if not flag or not done:
                continue

            if self.check_collision(new_node):
                # self.node_list.append(new_node)
                check, timeCollision =  self.check_dynamic_collision(new_node)
                if check:
                    self.node_list.append(new_node)
                    # print("Add new node")
                # elif timeCollision:
                #     tx = nearest_node.x_r + cos(nearest_node.theta_r)
                #     ty = nearest_node.y_r + sin(nearest_node.theta_r)
                #     ttheta = nearest_node.theta_r
                #     tv = 0
                #     tst = 0
                #     new_node = self.Node(tx, ty, ttheta, tv, tst)
                #     done, steering_time, sim_time = self.steer(nearest_node, new_node, self.agent)
                #     # print("######done!!!!!!")
                #     if done:
                #         if self.check_collision(new_node):
                #             check, timeCollision =  self.check_dynamic_collision(new_node)
                #             if check:
                #                 self.node_list.append(new_node)
                #             if self.animation:
                #                 self.draw_graph(new_node)
                #                 rnd_node = new_node
                            # print("WithoutCollision!!!!!!")
                            # print("time_node: ", time_node.x_r, " ", time_node.y_r, " ", time_node.theta_r, " ", time_node.v_r, " ", time_node.st_r, " ", time_node.time)
            if self.animation:
                self.draw_graph(rnd_node)
            
            
           
            if self.calc_dist_to_goal(self.node_list[-1].x,
                                    self.node_list[-1].y) <= self.radius:

                done, steering_time, sim_time = self.steer(self.node_list[-1], self.end, self.agent, goal=True)
                self.steering_time += steering_time
                self.simulation += sim_time
                delta_distance = hypot(self.end.x_r - self.end.x, self.end.y_r - self.end.y)
                delta_orientation = abs(normalizeAngle(self.end.theta - self.end.theta_r))
                # print(f"delta_orientation: {delta_orientation}")
                if not done:
                    self.end.clear()
                    # print("### Error ###")
                    continue
                if  not self.dwa:
                    if (delta_distance > 1.) or (delta_orientation > math.pi / 12.):
                        self.end.clear()
                        print("delta_orientation:", delta_orientation * 180 / math.pi)
                        print("### Error ###")
                        continue
                else:
                    # if (delta_distance > 1.) or (delta_orientation > math.pi / 4.):
                    #     self.end.clear()
                    print("DWA delta_orientation:", delta_orientation * 180 / math.pi)
                    # print("### Error ###")
                    # continue


                if self.check_collision(self.end):
                    check, timeCollision =  self.check_dynamic_collision(self.end)
                    if check:
                        if self.animation:
                            self.draw_graph(self.end)
                        
                        # self.draw_graph(self.end)
                        # if self.end.parent:
                        #     plt.plot(self.end.path_x, self.end.path_y, "-r")
                        #     for i in range(len(self.end.path_x)):
                        #         drawBB([self.end.path_x[i], self.end.path_y[i], self.end.path_theta[i]], color="-r")
                        # plt.pause(1.0)
                        # self.end.clear()

                        return self.generate_final_course(len(self.node_list) - 1)
                    else:
                        self.end.clear()
                else:
                    # self.draw_graph(self.end)
                    # plt.arrow(self.end.x, self.end.y, 5 * cos(self.end.theta), 5 * sin(self.end.theta), head_width=1.0, color='red')
                    # plt.plot(self.end.x_r, self.end.y_r, "mH")
                    # plt.arrow(self.end.x_r, self.end.y_r, 5 * cos(self.end.theta_r), 5 * sin(self.end.theta_r), head_width=1.0, color='blue')
                    # if self.end.parent:
                    #     plt.plot(self.end.path_x, self.end.path_y, "-m")
                    #     for i in range(len(self.end.path_x)):
                    #         drawBB([self.end.path_x[i], self.end.path_y[i], self.end.path_theta[i]])
                    # plt.pause(1.0)
                    self.end.clear()

        return None, None  # cannot find path

    def frameCollision(self, x, y):
        if x < 0 or x > self.width or y < 0 or y > self.height:
            return True
        return False
    
    def generateNewNode(self, from_node, to_node):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        extend_length = self.expand_dis

        if extend_length > d:
            extend_length = max(d, 3)
        
        x = from_node.x_r + extend_length * math.cos(theta) 
        y = from_node.y_r + extend_length * math.sin(theta)

        to_node.x = x
        to_node.y = y

        if not self.random:    
            theta += degToRad(np.random.randint(-self.rnd_delta_angle, self.rnd_delta_angle + 1))
            v = kmToM(np.random.randint(self.rnd_min_velocity, self.rnd_max_velocity + 1))        
            st = degToRad(np.random.randint(0, 10 + 1))
            to_node.theta = normalizeAngle(theta)
            to_node.v = v
            to_node.st = st
        
        if self.frameCollision(x, y) or not self.check_collision(to_node, rrt=True) \
                or self.calc_dist_to_goal(x, y) < 10:
            # print(False, " None")
            return False, None

        return True, to_node

    def steer(self, from_node, new_node, agent, goal=False):
        # steering_time = 0 
        """POSQ"""
        # sx, sy, stheta, sv, sst = from_node.x_r, from_node.y_r, from_node.theta_r, from_node.v_r, from_node.st_r
        # gx, gy, gtheta, gv, gst = new_node.x, new_node.y, new_node.theta, new_node.v, new_node.st_r         
        # print(done)
        # print(len(lst_params))
        """RL"""
        ## we need to transfrom the nodes and inverse transform
        
        sx, sy, stheta, sv, sst = from_node.x_r, from_node.y_r, from_node.theta_r, from_node.v_r, from_node.st_r
        gx, gy, gtheta, gv, gst = new_node.x, new_node.y, new_node.theta, new_node.v, new_node.st_r
        
        if (new_node.x == self.end.x and new_node.y == self.end.y and new_node.theta == self.end.theta):
            goal = True
        
        if self.rl:
            # transform = Transformation()
            # start_transform, goal_transform = transform.rotate([sx, sy, stheta], [gx, gy, gtheta])
            # start_transform.append(sv)
            # goal_transform.append(gv)
            # # steering angles
            # start_transform.append(sst)
            # goal_transform.append(gst)
            
            # # plt.plot(sx, sy, "rH")
            # # plt.arrow(sx, sy, 5 * cos(stheta), 5 * sin(stheta), head_width=1.0, color='red')
            # drawBB(start_transform, color="-r")
            # # plt.plot(gx, gy, "bH")
            # # plt.arrow(gx, gy, 5 * cos(gtheta), 5 * sin(gtheta), head_width=1.0, color='blue')
            # drawBB(goal_transform, color="-c")
            # self.draw_obstacles()
            # plt.show()
            # print(f"start_transform: {start_transform}")
            # print(f"goal_transform: {goal_transform}")
            # t = time.process_time()
            # done, lst_params = validateRRT(agent, valTasks, goal=goal)
            # new_node.time
            if from_node.time == 0:
                node_time = 0
            else:
                node_time = from_node.time
                # print(f"from_node.time {from_node.time}")
            dyn_trajectories = []
            node_time *= 10
            # self.dyn_trajectories[time * 10]
            # print(f"node_time: {node_time}")
            # print("self.dyn_trajectories", len(self.dyn_trajectories))
            for i in range(len(self.dyn_trajectories)):
                # print("self.dyn_trajectories[i][-1]", self.dyn_trajectories[i][-1])
                if node_time < len(self.dyn_trajectories[i]):
                    dyn_trajectories.append(self.dyn_trajectories[i][int(node_time):])
                else:
                    # print([self.dyn_trajectories[i][-1]])
                    dyn_trajectories.append([self.dyn_trajectories[i][-1]])
            
            start_transform = [sx, sy, stheta, sv, sst]
            goal_transform = [gx, gy, gtheta, gv, gst]
            valTasks = [(start_transform, goal_transform)]

            start_time = time.time()
            done, lst_params, steering_time = getTrajectory(self.env, agent, valTasks, obstacle_map=deepcopy(self.obstacles), dyn_trajectories=dyn_trajectories, goal=goal, dwa=self.dwa)
            end_time = time.time()
            simulation_time = end_time - start_time
            # print(f"done {done}")
            # print(end_time - start_time)
            # print(f"done {done}")
            # print(f"collision {collision}")
            # print(f"lst_params {lst_params}")
            # self.time_steering += time.process_time() - t
        else:
            valTasks = [([sx, sy, stheta, sv, sst], [gx, gy, gtheta, gv, gst])]
            # t = time.process_time()
            start_time = time.time()
            done, lst_params = validatePOSQ(valTasks, toGoal=goal)
            # print("POSQ: ", done)
            end_time = time.time()
            steering_time = end_time - start_time
            simulation_time = steering_time
            # print(end_time - start_time)
            # self.time_steering += time.process_time() - t
            # done = True
            # path_x, path_y, path_yaw, mode, lengths = dubinsSteer([sx, sy, stheta, sv, sst], [gx, gy, gtheta, gv, gst])
            # lst_params = [[path_x[i], path_y[i], path_yaw[i], 5., 0.] for i in range(len(path_x))]

            # done = True
            # path_x, path_y, path_yaw, mode, lengths = reedsSheppSteer([sx, sy, stheta, sv, sst], [gx, gy, gtheta, gv, gst])
            # lst_params = [[path_x[i], path_y[i], path_yaw[i], 5., 0.] for i in range(len(path_x))]
            # print("print(path_x): ", path_x)
            # print("print(path_y): ", path_y)
            # print("print(path_yaw): ", path_yaw)

        if not done:
            new_node.parent = None
            # print("##########Return False")
            return False, steering_time, simulation_time
        
        t_init = from_node.time
        
        dt = 0.1
        for i in range(len(lst_params)):
            if i == 0:
                continue
            x1, y1, theta1, v1, st1 = lst_params[i]
            # if self.rl:
            #     x1, y1, theta1 = transform.inverseRotate(x1, y1, theta1)
            
            new_node.path_x.append(x1)
            # print(x1)
            new_node.path_y.append(y1)
            # print(y1)
            new_node.path_theta.append(theta1)
            new_node.path_v.append(v1)
            new_node.path_st.append(st1)
            new_node.path_t.append(t_init + dt * i)

        new_node.x_r = new_node.path_x[-1]
        new_node.y_r = new_node.path_y[-1]
        new_node.theta_r = new_node.path_theta[-1]
        new_node.v_r = new_node.path_v[-1]
        new_node.st_r = new_node.path_st[-1]
        new_node.time = new_node.path_t[-1]
        new_node.parent = from_node
        # print(from_node)
        # plt.plot(new_node.path_x, new_node.path_y, "-g")
        # print("##########Return True")
        return True, steering_time, simulation_time

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.theta, self.end.v, self.end.st]]
        path_r = [[self.end.x_r, self.end.y_r, self.end.theta_r, self.end.v_r, self.end.st_r]]
        node_path = []
        node_path.append(self.end)
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.theta, node.v, node.st])
            path_r.append([node.x_r, node.y_r, node.theta_r, node.v_r, node.st_r])
            node_path.append(node)
            node = node.parent
        path.append([node.x, node.y, node.theta, node.v, node.st])
        path_r.append([node.x_r, node.y_r, node.theta_r, node.v_r, node.st_r])
        node_path.append(node)
        node_path.reverse()
        
        trajectory_x = []
        trajectory_y = []
        trajectory_v = []
        trajectory_theta = []
        trajectory_st = []
        trajectory_t = []
        
        for i in range(len(node_path)):
            trajectory_x.extend(node_path[i].path_x)
            trajectory_y.extend(node_path[i].path_y)
            trajectory_theta.extend(node_path[i].path_theta)
            trajectory_v.extend(node_path[i].path_v)
            trajectory_st.extend(node_path[i].path_st)
            trajectory_t.extend(node_path[i].path_t)

        self.trajectory = [trajectory_x, trajectory_y, trajectory_theta, trajectory_v, trajectory_st, trajectory_t]   
        
        return path, path_r

    # def generate_final_course(self, goal_ind):
    #     path = [[self.end.x, self.end.y]]
    #     trajectory_x = []
    #     trajectory_y = []
    #     node = self.node_list[goal_ind]
    #     trajectory_x.extend(node.path_x)
    #     trajectory_y.extend(node.path_y)

    #     while node.parent is not None:
    #         trajectory_x.extend(node.path_x)
    #         trajectory_y.extend(node.path_y)
    #         path.append([node.x, node.y])
    #         node = node.parent
        
    #     path.append([node.x, node.y])
    #     trajectory_x.extend(node.path_x)
    #     trajectory_y.extend(node.path_y)
        
    #     self.trajectory = [trajectory_x, trajectory_y]
    #     # for i in range(len(trajectory_x)):
    #     #     self.trajectory.append([trajectory_x[i], trajectory_y[i]])

    #     return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        # print("get_random_node")
        if random.randint(0, 100) > self.goal_sample_rate:
            x = random.uniform(0, self.width)
            # print("x: ", x)
            y = random.uniform(0, self.height)
            # print("y: ", y)
            theta = degToRad(np.random.randint(-180, 180))
            v = kmToM(np.random.randint(0, 10 + 1))        
            st = 0
            # print("x: ", x, "y: ", y, "theta: ", theta) 
            rnd = self.Node(
                x,
                y,
                theta, # for angle
                v, # for velocity  
                st) # for steering
            
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.theta, self.end.v, self.end.st)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # plt.close('all')
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            # plt.plot(node.x, node.y, "cH")
            plt.arrow(node.x, node.y, 5 * cos(node.theta), 5 * sin(node.theta), head_width=1.0, color='cyan')
            # drawBB([node.x_r, node.y_r, node.theta])
            plt.plot(node.x_r, node.y_r, "bH")
            plt.arrow(node.x_r, node.y_r, 5 * cos(node.theta_r), 5 * sin(node.theta_r), head_width=1.0, color='blue')
            if node.parent:
                # print("parent!!!!!!")
                plt.plot(node.path_x, node.path_y, "-g")
                # print(len(node.path_x))
                # for i in range(len(node.path_x)):
                #     drawBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
            # print("NONOparent!!!!!!")
        self.draw_obstacles()
        # plt.plot([self.frame[(i + 1) % len(self.frame)][0] for i in range(len(self.frame) + 1)], [self.frame[(i + 1) % len(self.frame)][1] for i in range(len(self.frame) + 1)], '-r')
        # plt.plot([i for i in range(self.height)], [self.width -1 for _ in range(self.height)], "bH")
        # plt.plot([self.width -1 for _ in range(self.height)], [i for i in range(self.height)], "bH")
        # plt.plot([0 for _ in range(self.height)], [i for i in range(self.height)], "bH")
        # plt.plot([i for i in range(self.height)], [0 for _ in range(self.height)], "bH")

        # plt.plot(self.start.x, self.start.y, "rH")
        # plt.arrow(self.start.x, self.start.y, 5 * cos(self.start.theta), 5 * sin(self.start.theta), head_width=1.0, color='red')
        # plt.plot(self.end.x, self.end.y, "rH")
        # plt.arrow(self.end.x, self.end.y, 5 * cos(self.end.theta), 5 * sin(self.end.theta), head_width=1.0, color='red')
        #plt.axis("equal")
        #plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        # plt.show()
        plt.pause(0.1)
    
    def draw_obstacles(self, show_dyn_obst=True):
        # print(self.obstacles)
        plt.plot(self.start.x, self.start.y, "rH")
        plt.arrow(self.start.x, self.start.y, 5 * cos(self.start.theta), 5 * sin(self.start.theta), head_width=1.0, color='red')
        plt.plot(self.end.x, self.end.y, "bH")
        plt.arrow(self.end.x, self.end.y, 5 * cos(self.end.theta), 5 * sin(self.end.theta), head_width=1.0, color='blue')

        for obs in self.obstacles:
            bbObs = getBB(obs, ego=False)
            plt.plot([bbObs[(i + 1) % len(bbObs)][0] for i in range(len(bbObs) + 1)], [bbObs[(i + 1) % len(bbObs)][1] for i in range(len(bbObs) + 1)], '-k')
        if show_dyn_obst:
            for dyn_obst in self.dyn_trajectories:
                plt.plot(dyn_obst[0][0], dyn_obst[0][1], "yH")
                plt.plot([d[0] for d in dyn_obst], [d[1] for d in dyn_obst], "--")
        plt.plot([self.frame[(i + 1) % len(self.frame)][0] for i in range(len(self.frame) + 1)], [self.frame[(i + 1) % len(self.frame)][1] for i in range(len(self.frame) + 1)], '-r')

    def check_collision(self, node, rrt=False):
        if node is None:
            return False

        # return True
        # print([node.x, node.y, node.theta])
        if rrt or not self.rl:
            if (len(node.path_x) > 0):
                for i in range(len(node.path_x)):
                    if self.frameCollision(node.path_x[i], node.path_y[i]):
                        return False

            bb = getBB([node.x, node.y, node.theta])

            for obs in self.obstacles:
                # print(f"obs :{obs}")
                bbObs = getBB(obs, ego=False)
                if intersectPoint([node.x, node.y], bbObs):
                    # print("False1")
                    return False

                if intersect(bb, bbObs):
                    # print("False2")
                    return False

                # node = self.vehicle.shift_state(deepcopy(node))
                if (len(node.path_x) > 0):
                    for i in range(len(node.path_x)):
                        if intersectPoint([node.path_x[i], node.path_y[i]], bbObs):
                            return False
                        bb = getBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
                        if intersect(bb, bbObs):
                            return False
            
            return True
        else:
            return True

    def check_dynamic_collision(self, node):
        # dynamic trajectory always initializes in t = 0
        return True, None
        for dyn_trajectory in self.dyn_trajectories:
            dynamic_t_final = len(dyn_trajectory) * 0.1
            dynamic_t_init = 0
            if not (dynamic_t_final < node.path_t[0] or node.path_t[-1] < dynamic_t_init):
                lower_time_bound = max(node.path_t[0], dynamic_t_init)
                upper_time_bound = min(node.path_t[-1], dynamic_t_final)
                # print("#####lower_time_bound#####: ", lower_time_bound)
                # print("#####upper_time_bound#####: ", upper_time_bound)
                # we have intersection by time
                # if len(dyn_trajectory[lower_time_bound*10: upper_time_bound*10]) > 0:
                # for index in range(lower_time_bound*10, upper_time_bound*10+1):
                #     obs_bb = getBB(dyn_trajectory[index])
                for t, dyn in enumerate(dyn_trajectory):
                    time = t * 0.1
                    if lower_time_bound <= time or upper_time_bound >= time:
                        obs_bb = getBB(dyn)
                        for i in range(len(node.path_t)):
                            if abs(node.path_t[i] - time) < 0.2:
                                # if hypot(dyn[0] - node.path_x[i], dyn[1] - node.path_y[i]) < 5.0:
                                bb = getBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
                                if intersect(bb, obs_bb):
                                    # print("Collisions with dynamic obstacles")
                                    return False, True

                # for time, position in dyn_obs:
                #     if lower_time_bound <= time or upper_time_bound >= time:
                #         # print("time: ", time)
                #         # print("position: ", position)
                #         obs_bb = getBB([position[0], position[1], 0 * math.pi / 2.])
                #         for i in range(len(node.path_t)):  
                #             if node.path_t[i] - time == 0:
                #                 if hypot(position[0] - node.path_x[i], position[1] - node.path_y[i]) < 3.0:
                #                     bb = getBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
                #                     if intersect(bb, obs_bb):
                #                         return False, True
                        # if intersectPoint([node.x, node.y], obs_bb):
                        #     return False
                        # bb = getBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
                        # if intersectPoint(bb, obs_bb):
                        #     return False
        #     elif dyn_obs[-1][0] < node.path_t[0]:
        #         obs_bb = getBB([dyn_obs[-1][1][0], dyn_obs[-1][1][1], 0 * math.pi / 2.], ego=False)
        #         for i in range(len(node.path_x)):
        #             # if intersectPoint([node.path_x[i], node.path_y[i]], obs_bb) or self.frameCollision(node.path_x[i], node.path_y[i]):
        #             #     print("intersectTimeStop: ", [dyn_obs[-1][1][0], dyn_obs[-1][1][1]])
        #             #     return False
        #             bb = getBB([node.path_x[i], node.path_y[i], node.path_theta[i]])
        #             if intersect(bb, obs_bb):
        #                 # print("intersectPositionStop: ", [dyn_obs[-1][1][0], dyn_obs[-1][1][1]])
        #                 # print("intersectTimeStop: ", time)
        #                 return False, False
                
        # print("Does not collsion with dynamic obstacles")
        return True, None  # safe
    
    # def get_length_trajectory(self):
    #     length = 0
    #     for i in range(1, len(self.trajectory)):
    #         length += hypot(self.trajectory[i][1] - self.trajectory[i-1][1], self.trajectory[i][0] - self.trajectory[i-1][0])
        
    #     return length

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        
        # dlist = [((node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2) /  (2 * 100 ** 2) + abs(normalizeAngle(node.theta - rnd_node.theta)) / (2 * pi) / 25
        #           for node in node_list]
        A = np.array(dlist)
        if A.shape[0] > NEAREST:
            lst_index = list(np.argsort(A)[:NEAREST])   
        else:
            lst_index = list(np.argsort(A))
        
        # new_node_list = [(node_list[index], index) for index in lst_index]
        # new_node_list.sort(key=lambda x: x[0].time)
        # new_lst_index = [pair[1] for pair in new_node_list]
        # return new_lst_index

        return lst_index

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta