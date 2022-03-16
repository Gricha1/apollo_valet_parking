import math
import random
import matplotlib.pyplot as plt
import numpy as np
from .utilsPlanning import *
from EnvLib.line import *
import time

mark_size = 8
line_size = 2

class RRTGeometric:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.x_r = x
            self.y_r = y
            self.path_x = []
            self.path_y = []

        def clear(self):
            self.path_x = []
            self.path_y = []

    def __init__(self,
                 start,
                 goal,
                 agent,
                 env=None,
                 obstacles=[],
                 dyn_obstacles=[],
                 width=100,
                 height=40,
                 expand_dis=10.0,
                 goal_sample_rate=5,
                 max_iter=1500,
                 rl=True,
                 animation=True,
                 random=False,
                 smoothing=False):
        """
        Setting Parameter

        start: Start State [x, y]
        goal: Goal State [x, y]
        """
        self.env = env
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.dyn_obstacles = dyn_obstacles
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.rl = rl
        self.node_list = []
        self.trajectory = []
        self.time_steering = 0.
        self.number_samples = 0
        self.number_success_samples = 0
        self.animation = animation
        self.random = random
        self.resolution = 1.0
        self.frame = [[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]]
        self.lst_bbObs = []
        self.with_smoothing = smoothing
        for obst in self.obstacles:
            bbObs = getBB(obst, ego=False)
            self.lst_bbObs.append(bbObs)

    def planning(self):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            success, rnd_node = self.generateNewNode(nearest_node, rnd_node)
            
            #Succes checks for collision
            if not success:
                continue

            new_node = self.steer(nearest_node, rnd_node)

            self.node_list.append(new_node)

            if self.animation:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                
                success, self.end = self.generateNewNode(self.node_list[-1], self.end)
                if not success:
                    continue
                self.end = self.steer(self.node_list[-1], self.end)
                # if self.check_collision(final_node, self.obstacle_list):
                path, path_r = self.generate_final_course(len(self.node_list) - 1)
                if not self.with_smoothing:
                    return path, path_r
                else:
                    path = self.path_smoothing(path)
                    return path, path


            if self.animation:
                self.draw_graph(rnd_node)

        return None  
    
    def generateNewNode(self, from_node, to_node):

        d, theta = self.calc_distance_and_angle(from_node, to_node)
        
        extend_length = self.expand_dis
        if extend_length > d:
            extend_length = d
        
        x = from_node.x_r + extend_length * math.cos(theta)
        y = from_node.y_r + extend_length * math.sin(theta)
        
        new_node = self.Node(x, y)
        
        if self.collision(new_node, theta=theta):
            return False, None

        if self.segment_collision(from_node, new_node):
            return False, None
        
        return True, new_node

    def steer(self, from_node, new_node):

        distance, theta = self.calc_distance_and_angle(from_node, new_node)
        n_expand = math.floor(distance / self.resolution)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        # new_node.path_x.append(from_node.x)
        # new_node.path_y.append(from_node.y)
        for i in range(n_expand):
            new_node.path_x.append(from_node.x + i * self.resolution * cos_theta)
            new_node.path_y.append(from_node.y + i * self.resolution * sin_theta)
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)

        new_node.x_r = new_node.x
        new_node.y_r = new_node.y
        new_node.parent = from_node
        
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        path_r = [[self.end.x_r, self.end.y_r]]
        node_path = []
        node_path.append(self.end)
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            path_r.append([node.x_r, node.y_r])
            node_path.append(node)
            node = node.parent
        path.append([node.x, node.y])
        path_r.append([node.x_r, node.y_r])
        node_path.append(node)
        node_path.reverse()
        
        trajectory_x = []
        trajectory_y = []
        
        for i in range(len(node_path)):
            trajectory_x.extend(node_path[i].path_x)
            trajectory_y.extend(node_path[i].path_y)

        self.trajectory = [trajectory_x, trajectory_y]   
        
        return path, path_r

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(0, self.width),
                random.uniform(0, self.height))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_obstacles(self):
        # print(self.obstacles)
        for obs in self.obstacles:
            bbObs = getBB(obs, ego=False)
            plt.plot([bbObs[(i + 1) % len(bbObs)][0] for i in range(len(bbObs) + 1)], [bbObs[(i + 1) % len(bbObs)][1] for i in range(len(bbObs) + 1)], '-k')

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            plt.plot(node.x_r, node.y_r, "bH")
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
            #     print("Has a parent")
            # else:
            #     print("Does not have a parent")
        self.draw_obstacles()
        plt.plot(self.start.x, self.start.y, "rH")
        plt.plot(self.end.x, self.end.y, "rH")
        plt.grid(True)
        plt.pause(0.01)

    def collision(self, node, theta=None):
        if node is None:
            return False

        for bbObs in self.lst_bbObs:
            # bbObs = getBB(obs, ego=False)
            if intersectPoint([node.x, node.y], bbObs):
                return True
            if theta != None:
                bb = getBB([node.x, node.y, theta])
                if intersectPolygons(bb, bbObs, rl=False):
                    return True
        return False  # safe
    
    def segment_collision(self, from_node, to_node):
        p1 = Point(from_node.x, from_node.y)
        q1 = Point(to_node.x, to_node.y)
        for bbObs in self.lst_bbObs:
            # bbObs = getBB(obst, ego=False)
            # print(f"bbObs {bbObs}")
            for i in range(len(bbObs)):
                v1 = bbObs[(i) % len(bbObs)]
                p2 = Point(v1[0], v1[1]) 
                # print(f"v1 {v1}")
                v2 = bbObs[(i + 1) % len(bbObs)] 
                q2 = Point(v2[0], v2[1])
                # print(f"v2 {v2}")
                if(doIntersect(p1, q1, p2, q2)):
                    return True
        return False
    
    def get_length_trajectory(self):
        length = 0
        for i in range(1, len(self.trajectory)):
            length += math.hypot(self.trajectory[i][1] - self.trajectory[i-1][1], self.trajectory[i][0] - self.trajectory[i-1][0])
        
        return length
    
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    
    @staticmethod
    def get_path_length(path):
        le = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d

        return le

    @staticmethod
    def get_target_point(path, targetL):
        le = 0
        ti = 0
        lastPairLen = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d
            if le >= targetL:
                ti = i - 1
                lastPairLen = d
                break

        partRatio = (le - targetL) / lastPairLen

        x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
        y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

        return [x, y, ti]

    def path_smoothing(self, path, max_iter=100):
        le = self.get_path_length(path)

        for i in range(max_iter):
            # Sample two points
            pickPoints = [random.uniform(0, le), random.uniform(0, le)]
            pickPoints.sort()
            first = self.get_target_point(path, pickPoints[0])
            second = self.get_target_point(path, pickPoints[1])

            if first[2] <= 0 or second[2] <= 0:
                continue

            if (second[2] + 1) > len(path):
                continue

            if second[2] == first[2]:
                continue

            # collision check
            # print(f"first {first}")
            # print(f"second {second}")
            p1 = Point(first[0], first[1])
            q1 = Point(second[0], second[1])

            theta = math.atan2(q1.y - p1.y, q1.x - p1.x)
            if self.segment_collision(p1, q1):
                # print("Not smoothing")
                continue
            if self.collision(p1, theta=theta):
                continue
            if self.collision(q1, theta=theta):
                continue
            # print("Smoothing")
            newPath = []
            print(f"first {first}")
            print("PAHT: ", path[first[2]])
            print(f"second {second}")
            print("PAHT: ", path[second[2]])

            newPath.extend(path[:first[2] + 1])
            newPath.append([first[0], first[1]])

            # if math.hypot(q1.y - p1.y, q1.x - p1.x) > 20:
            #     newPath.append([(first[0] + second[0]) / 2., (first[1] + second[1]) / 2.])
            #     newPath.append([second[0], second[1]])
            #     newPath.extend(path[second[2] + 2:])
            # # Create New path
            # else:
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            le = self.get_path_length(path)

        return path