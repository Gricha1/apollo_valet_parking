import gym
import matplotlib.pyplot as plt

from modules.tools.custom_2.planning.generateMap import generateTasks
from modules.tools.custom_2.EnvLib.line import *
from math import pi
import numpy as np
from modules.tools.custom_2.EnvLib.Vec2d import Vec2d
from modules.tools.custom_2.EnvLib.utils import *
from math import cos, sin, tan
# from copy import deepcopy
from scipy.spatial import cKDTree
from modules.tools.custom_2.planning.utilsPlanning import *
import time


class State:
    def __init__(self, x, y, theta, v, steer):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.steer = steer
        self.width = 0
        self.length = 0

class VehicleConfig:
    def __init__(self, car_config):
        self.length = car_config["length"]
        self.width = car_config["width"]
        self.wheel_base = car_config["wheel_base"]
        self.safe_eps = car_config["safe_eps"]
        self.max_steer = degToRad(car_config["max_steer"])
        self.max_vel = car_config["max_vel"]
        self.min_vel = car_config["min_vel"]
        self.max_acc = car_config["max_acc"]
        self.max_ang_vel = car_config["max_ang_vel"]
        self.max_ang_acc = car_config["max_ang_acc"]
        self.delta_t = car_config["delta_t"]
        self.rear_to_center = (self.length - self.wheel_base) / 2.
        self.min_dist_to_check_collision = math.hypot(self.rear_to_center + self.length / 2., self.width / 2.)
        
    
    def dynamic(self, state, action):
        a = action[0]
        Eps = action[1]
        dt = self.delta_t
        self.a = a
        self.Eps = Eps
        
        dV = a * dt
        V = state.v + dV
        overSpeeding = V > self.max_vel or V < self.min_vel
        V = np.clip(V, self.min_vel, self.max_vel)

        dv_s = Eps * dt
        self.v_s = self.v_s + dv_s
        dsteer = self.v_s * dt
        steer = normalizeAngle(state.steer + dsteer)
        overSteering = abs(steer) > self.max_steer
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        w = (V * np.tan(steer) / self.wheel_base)
        dtheta = w * dt
        theta = normalizeAngle(state.theta + dtheta)

        dx = V * np.cos(theta) * dt
        dy = V * np.sin(theta) * dt
        x = state.x + dx
        y = state.y + dy

        new_state = State(x, y, theta, V, steer)

        return new_state, overSpeeding, overSteering
    
    '''
    def dynamic(self, state, action):
        a = action[0]
        Eps = action[1]
        dt = self.delta_t
        
        dV = a * dt
        V = state.v + dV
        overSpeeding = V > self.max_vel or V < self.min_vel
        V = np.clip(V, self.min_vel, self.max_vel)

        dw = Eps * dt
        dsteer = dw * dt
        steer = normalizeAngle(state.steer + dsteer)
        overSteering = abs(steer) > self.max_steer
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        dtheta = (V * np.tan(steer) / self.wheel_base) * dt
        theta = normalizeAngle(state.theta + dtheta)

        dx = V * np.cos(theta) * dt
        dy = V * np.sin(theta) * dt
        x = state.x + dx
        y = state.y + dy

        new_state = State(x, y, theta, V, steer)

        return new_state, overSpeeding, overSteering
    '''
    
    def shift_state(self, state, toCenter=False):
        l = self.length / 2
        shift = l - self.rear_to_center
        shift = -shift if toCenter else shift
        new_state = State(state.x + shift * cos(state.theta), state.y + shift * sin(state.theta), state.theta, state.v, state.steer)
        return new_state

class ObsEnvironment(gym.Env):
    def __init__(self, config):
        env_config = config["our_env_config"]
        self.reward_config = config["reward_config"]
        self.goal = None
        self.current_state = None
        self.old_state = None
        self.last_action = [0., 0.]
        self.obstacle_segments = []
        self.dyn_obstacle_segments = []
        self.last_observations = []
        self.hardGoalReached = False
        self.stepCounter = 0
        self.vehicle = config['vehicle_config']
        self.trainTasks = config['tasks']
        self.valTasks = config['valTasks']
        self.maps_init = config['maps']
        self.maps = dict(config['maps'])
        self.alpha = env_config['alpha']
        self.max_steer = env_config['max_steer']
        self.max_dist = env_config['max_dist']
        self.min_dist = env_config['min_dist']
        self.min_vel = env_config['min_vel']
        self.max_vel = env_config['max_vel']
        self.min_obs_v = env_config['min_obs_v']
        self.max_obs_v = env_config['max_obs_v']
        self.HARD_EPS = env_config['HARD_EPS']
        self.SOFT_EPS = env_config['SOFT_EPS']
        self.ANGLE_EPS = degToRad(env_config['ANGLE_EPS'])
        self.SPEED_EPS = kmToM(env_config['SPEED_EPS'])
        self.STEERING_EPS = degToRad(env_config['STEERING_EPS'])
        self.MAX_DIST_LIDAR = env_config['MAX_DIST_LIDAR']
        self.UPDATE_SPARSE = env_config['UPDATE_SPARSE']
        self.view_angle = degToRad(env_config['view_angle'])
        self.hard_constraints = env_config['hard_constraints']
        self.soft_constraints = env_config['soft_constraints']
        self.affine_transform = env_config['affine_transform']
        self.with_potential = env_config['reward_with_potential']
        self.frame_stack = env_config['frame_stack']
        self.bias_beam = env_config['bias_beam']
        self.n_beams = env_config['n_beams']
        self.dynamic_obstacles = []
        self.dyn_acc = 0
        self.dyn_ang_vel = 0
        self.collision_time = 0
        self.angle_space = np.linspace(-self.view_angle, self.view_angle, self.n_beams)
        self.reward_weights = [
            self.reward_config["collision"],
            self.reward_config["goal"],
            self.reward_config["timeStep"],
            self.reward_config["distance"],
            self.reward_config["overSpeeding"],
            self.reward_config["overSteering"]
        ]
        self.unionTask = env_config['union']
        if self.unionTask:
            self.second_goal = State(config["second_goal"][0], 
                                config["second_goal"][1],
                                config["second_goal"][2],
                                config["second_goal"][3],
                                config["second_goal"][4])

        other_features = len(self.getDiff(State(0, 0, 0, 0, 0)))
        state_min_box = [-np.inf for _ in range(47)] * self.frame_stack + [-np.inf] * self.frame_stack
        state_max_box = [np.inf for _ in range(47)] * self.frame_stack + [np.inf] * self.frame_stack                                                                           
        obs_min_box = np.array(state_min_box)
        obs_max_box = np.array(state_max_box)
        self.observation_space = gym.spaces.Box(obs_min_box, obs_max_box, dtype=np.float32)
        self.action_space = gym.spaces.Box(low=np.array([-self.vehicle.max_acc, 
                                -self.vehicle.max_ang_acc]), 
            high=np.array([self.vehicle.max_acc, self.vehicle.max_ang_acc]), dtype=np.float32)
        self.lst_keys = list(self.maps.keys())
        index = np.random.randint(len(self.lst_keys))
        self.map_key = self.lst_keys[index]
        self.obstacle_map = self.maps[self.map_key]
            
    def getBB(self, state, width=2.0, length=3.8, ego=True):
        x = state.x
        y = state.y
        angle = state.theta
        if ego:
            w = self.vehicle.width / 2
            l = self.vehicle.length / 2
        else:
            w = width
            l = length
        BBPoints = [(-l, -w), (l, -w), (l, w), (-l, w)]
        vertices = []
        sinAngle = math.sin(angle)
        cosAngle = math.cos(angle)
        for i in range(len(BBPoints)):
            new_x = cosAngle * (BBPoints[i][0]) - sinAngle * (BBPoints[i][1])
            new_y = sinAngle * (BBPoints[i][0]) + cosAngle * (BBPoints[i][1])
            vertices.append(Point(new_x + x, new_y + y))
            
        segments = [(vertices[(i) % len(vertices)], \
                    vertices[(i + 1) % len(vertices)]) for i in range(len(vertices))]
        
        return segments


    def __sendBeam(self, state, angle, nearestObstacles=None, with_angles=False, lst_indexes=[]):
        if nearestObstacles is None:
            nearestObstacles = list(self.obstacle_segments)
            nearestObstacles.extend(self.dyn_obstacle_segments)
        
        angle = normalizeAngle(angle + state.theta)
        new_x = state.x + self.MAX_DIST_LIDAR * cos(angle)
        new_y = state.y + self.MAX_DIST_LIDAR * sin(angle)
        p1 = Point(state.x, state.y)
        q1 = Point(new_x, new_y)
        min_dist = self.MAX_DIST_LIDAR
        for i, obstacles in enumerate(nearestObstacles):
            for obst_with_angles in obstacles:
                if with_angles:
                    angle1, angle2 = obst_with_angles[0]
                    p2, q2 = obst_with_angles[1]
                    if not angleIntersection(angle1, angle2, angle):
                        continue
                else:
                    p2, q2 = obst_with_angles

                if(doIntersect(p1, q1, p2, q2)):
                    beam = Line(p1, q1)
                    segment = Line(p2, q2)
                    intersection = beam.isIntersect(segment)
                    distance = math.hypot(p1.x - intersection.x, p1.y - intersection.y)
                    min_dist = min(min_dist, distance)
                    if (distance < self.vehicle.min_dist_to_check_collision):
                        if i not in lst_indexes and i < len(self.obstacle_segments):
                            lst_indexes.append(i)
                    
        return min_dist
    
    def getRelevantSegments(self, state, with_angles=False):
        relevant_obstacles = []
        obstacles = list(self.obstacle_segments)
        obstacles.extend(self.dyn_obstacle_segments)
        for obst in obstacles:
            new_segments = []
            for segment in obst:
                d1 = math.hypot(state.x - segment[0].x, state.y - segment[0].y)
                d2 = math.hypot(state.x - segment[1].x, state.y - segment[1].y)
                new_segments.append((min(d1, d2), segment)) 
            new_segments.sort(key=lambda s: s[0])
            new_segments = [pair[1] for pair in new_segments[:2]]
            if not with_angles:
                relevant_obstacles.append(new_segments)
            else:
                new_segments_with_angle = []
                angles = []
                for segment in new_segments:
                    angle1 = math.atan2(segment[0].y - state.y, segment[0].x - state.x)
                    angle2 = math.atan2(segment[1].y - state.y, segment[1].x - state.x)
                    min_angle = min(angle1, angle2)
                    max_angle = max(angle1, angle2)
                    new_segments_with_angle.append(((min_angle, max_angle), segment))
                    angles.append((min_angle, max_angle))
                if angleIntersection(angles[0][0], angles[0][1], angles[1][0]) and \
                    angleIntersection(angles[0][0], angles[0][1], angles[1][1]):
                    relevant_obstacles.append([new_segments_with_angle[0]])
                elif angleIntersection(angles[1][0], angles[1][1], angles[0][0]) and \
                    angleIntersection(angles[1][0], angles[1][1], angles[0][1]):
                    relevant_obstacles.append([new_segments_with_angle[1]])
                else:
                    relevant_obstacles.append(new_segments_with_angle)
                    
        return relevant_obstacles

    def __getObservation(self, state):
        new_beams = []
        lst_indexes = []
        if len(self.obstacle_segments) > 0 or len(self.dyn_obstacle_segments) > 0:
            with_angles=True
            nearestObstacles = self.getRelevantSegments(state, with_angles=with_angles)
            for angle in self.angle_space:
                beam = self.__sendBeam(state, angle, nearestObstacles, 
                                with_angles=with_angles, lst_indexes=lst_indexes)
                new_beams.append(beam - self.bias_beam)
        else:
            for angle in self.angle_space:
                new_beams.append(self.MAX_DIST_LIDAR - self.bias_beam)

        if len(self.last_observations) == 0:
            for _ in range(self.frame_stack - 1):
                self.last_observations.extend(new_beams)
                self.last_observations.extend(self.getDiff(state))
                self.last_observations.append(self.task)

        self.last_observations.extend(new_beams)
        obs_from_state = self.getDiff(state)
        self.last_observations.extend(obs_from_state)
        self.last_observations.append(self.task)
        observation = self.last_observations
        self.last_observations = self.last_observations[len(new_beams) 
                                                    + len(obs_from_state) + 1:]

        return np.array(observation, dtype=np.float32), np.min(new_beams), lst_indexes

    def getDiff(self, state):
        if self.goal is None:
            self.goal = state
        delta = []
        dx = self.goal.x - state.x
        dy = self.goal.y - state.y
        dtheta = self.goal.theta - state.theta
        dv = self.goal.v - state.v
        dsteer = self.goal.steer - state.steer
        theta = state.theta
        v = state.v
        steer = state.steer
        delta.extend([dx, dy, dtheta, dv, dsteer, theta, v, steer])
        
        return delta

    def transformTask(self, from_state, goal_state, obstacles, dynamic_obstacles=[]):
        
        if self.affine_transform:
            sx, sy, stheta, sv, sst = from_state
            gx, gy, gtheta, gv, gst = goal_state
            self.transform = Transformation()
            start_transform, goal_transform = self.transform.rotate([sx, sy, stheta], [gx, gy, gtheta])
            start_transform.append(sv)
            goal_transform.append(gv)
            start_transform.append(sst)
            goal_transform.append(gst)

            new_obstacle_map = []
            for index in range(len(obstacles)):
                x, y, theta, width, length = obstacles[index]
                state = self.transform.rotateState([x, y, theta])
                new_obstacle_map.append([state[0], state[1],  state[2], width, length])
            self.obstacle_map = new_obstacle_map

            new_dyn_obstacles = []
            for index in range(len(dynamic_obstacles)):
                x, y, theta, v, st = dynamic_obstacles[index]
                state = self.transform.rotateState([x, y, theta])
                new_dyn_obstacles.append(State(state[0], state[1], state[2], v, st))
                
            self.dynamic_obstacles = new_dyn_obstacles
        else:
            start_transform = list(from_state)
            goal_transform = list(goal_state)
            new_obstacle_map = []
            for index in range(len(obstacles)):
                state = obstacles[index]
                new_obstacle_map.append([state[0], state[1],  state[2], state[3], state[4]])
            self.obstacle_map = new_obstacle_map

            new_dyn_obstacles = []
            for index in range(len(dynamic_obstacles)):
                state = dynamic_obstacles[index]
                new_dyn_obstacles.append(State(state[0], state[1], state[2], state[3], state[4]))
                
            self.dynamic_obstacles = new_dyn_obstacles

        start = State(start_transform[0], start_transform[1], start_transform[2], start_transform[3], start_transform[4])
        goal = State(goal_transform[0], goal_transform[1], goal_transform[2], goal_transform[3], goal_transform[4])
        
        return start, goal 

    def generateSimpleTask(self, obstacles=[]):
        if (len(obstacles) > 0):
            train_tasks = generateTasks(obstacles)
            start, goal = train_tasks[0]
        else:
            start_x = 0
            start_y = 0
            goal_x = np.random.randint(self.min_dist, self.max_dist + 1)
            goal_y = 0
            start_theta = degToRad(np.random.randint(-self.alpha, self.alpha + 1))
            goal_theta = degToRad(np.random.randint(-self.alpha, self.alpha + 1))
            start_v = 0
            goal_v = np.random.randint(self.min_vel, self.max_vel + 1)
            start_steer = degToRad(np.random.randint(-self.max_steer, self.max_steer + 1))
            goal_steer = 0
            start = [start_x, start_y, start_theta, start_v, start_steer]
            goal = [goal_x, goal_y, goal_theta, goal_v, goal_steer]
        
        return (start, goal)

    def setTask(self, tasks, idx, obstacles, rrt):
        if len(tasks) > 0:
            i = np.random.randint(len(tasks)) if idx is None else idx
            current_task = tuple(tasks[i])
            if(len(current_task) == 2):
                current, goal = current_task
            else:
                #print("DEBUG", current_task) #DEBUG
                current, goal, dynamic_obstacles = current_task
                if not rrt:
                    if (np.random.randint(3) > 0):
                        for dyn_obst in dynamic_obstacles:
                            self.dynamic_obstacles.append(dyn_obst)
                else:
                    for dyn_obst in dynamic_obstacles:
                        self.dynamic_obstacles.append(dyn_obst)
        else:
            current, goal = self.generateSimpleTask(obstacles)

        self.current_state, self.goal = self.transformTask(current, goal, obstacles, self.dynamic_obstacles)
        self.old_state = self.current_state

    def reset(self, idx=None, fromTrain=True, val_key=None, rrt=False):
        self.maps = dict(self.maps_init)
        self.hardGoalReached = False
        self.stepCounter = 0
        self.last_observations = []
        self.last_action = [0., 0.]
        self.obstacle_segments = []
        self.dyn_obstacle_segments = []
        self.dynamic_obstacles = []
        self.dyn_acc = 0
        self.dyn_ang_vel = 0
        self.dyn_ang_acc = 0
        #self.vehicle.car_w = 0
        self.vehicle.v_s = 0
        self.vehicle.Eps = 0
        self.vehicle.a = 0
        self.collision_time = 0
        if self.unionTask:    
            self.first_goal_reached = False
        else:
            self.first_goal_reached = True
        

        if fromTrain:
            index = np.random.randint(len(self.lst_keys))
            self.map_key = self.lst_keys[index]
            self.obstacle_map = self.maps[self.map_key]
            tasks = self.trainTasks[self.map_key]
            self.setTask(tasks, idx, self.obstacle_map, rrt)
        else:
            self.map_key = val_key
            self.obstacle_map = self.maps[self.map_key]
            tasks = self.valTasks[self.map_key]
            self.setTask(tasks, idx, self.obstacle_map, rrt)
        
        for obstacle in self.obstacle_map:
            obs = State(obstacle[0], obstacle[1], obstacle[2], 0, 0)
            width = obstacle[3]
            length = obstacle[4]
            self.obstacle_segments.append(self.getBB(obs, width=width, length=length, ego=False))
        self.dyn_obstacle_segments = []
        for dyn_obst in self.dynamic_obstacles:
            if math.hypot(self.current_state.x - dyn_obst.x, self.current_state.y - dyn_obst.y) < (self.MAX_DIST_LIDAR + self.vehicle.min_dist_to_check_collision):
                center_dyn_obst = self.vehicle.shift_state(dyn_obst)
                self.dyn_obstacle_segments.append(self.getBB(center_dyn_obst))

        if self.goal.theta != degToRad(90):
            self.task = 1 #forward task
        else:
            self.task = -1 #backward task

        observation, _, _ = self.__getObservation(self.current_state)
        
        return observation
    
    def __reward(self, current_state, new_state, goalReached, collision, overSpeeding, overSteering):
        previous_delta = self.__goalDist(current_state)
        new_delta = self.__goalDist(new_state)
        reward = []

        reward.append(-1 if collision else 0)

        if goalReached:
            reward.append(1)
        else:
            reward.append(0)

        if not (self.stepCounter % self.UPDATE_SPARSE):
            if (new_delta < 0.5):
                new_delta = 0.5
            reward.append(-1)
            if self.with_potential:
                reward.append((previous_delta - new_delta) / new_delta)
            else:
                reward.append(previous_delta - new_delta)
            reward.append(-1 if overSpeeding else 0)
            reward.append(-1 if overSteering else 0)
        else:
            reward.append(0)
            reward.append(0)
            reward.append(0)
            reward.append(0)

        # reward = np.array(reward)

        return np.matmul(self.reward_weights, reward)

    def isCollision(self, state, min_beam, lst_indexes=[]):
        
        if (self.vehicle.min_dist_to_check_collision < min_beam):
            return False

        if len(self.obstacle_segments) > 0 or len(self.dyn_obstacle_segments) > 0:
            bounding_box = self.getBB(state)
            for i, obstacle in enumerate(self.obstacle_segments):
                if i in lst_indexes:
                    if (intersectPolygons(obstacle, bounding_box)):
                        return True
                    
            for obstacle in self.dyn_obstacle_segments:
                mid_x = (obstacle[0][0].x + obstacle[1][1].x) / 2.
                mid_y = (obstacle[0][0].y + obstacle[1][1].y) / 2.
                distance = math.hypot(mid_x - state.x, mid_y - state.y)
                if (distance > (self.vehicle.min_dist_to_check_collision)):
                    continue
                if (intersectPolygons(obstacle, bounding_box)):
                    return True
            
        return False

    def __goalDist(self, state):
        return math.hypot(self.goal.x - state.x, self.goal.y - state.y )

    def step(self, action, next_dyn_states=[]):
        info = {}
        isDone = False
        new_state, overSpeeding, overSteering = self.vehicle.dynamic(self.current_state, action)
        
        if len(self.dynamic_obstacles) > 0:
            dynamic_obstacles = []
            if not (self.stepCounter % self.UPDATE_SPARSE):
                self.dyn_acc = np.random.randint(-self.vehicle.max_acc, self.vehicle.max_acc + 1)
                self.dyn_ang_acc = np.random.randint(-self.vehicle.max_ang_acc, self.vehicle.max_ang_acc)

            for index, dyn_obst in enumerate(self.dynamic_obstacles):
                if len(next_dyn_states) > 0:
                    x, y, theta, v, st = next_dyn_states[index]
                    state = self.transform.rotateState([x, y, theta])
                    new_dyn_obst = State(state[0], state[1], state[2], v, st)
                else:
                    new_dyn_obst, _, _ = self.vehicle.dynamic(dyn_obst, [self.dyn_acc, self.dyn_ang_acc])
                
                dynamic_obstacles.append(new_dyn_obst)
            self.dynamic_obstacles = dynamic_obstacles
            
            self.dyn_obstacle_segments = []
            for dyn_obst in self.dynamic_obstacles:
                distance = math.hypot(new_state.x - dyn_obst.x, new_state.y - dyn_obst.y)
                if distance < (self.MAX_DIST_LIDAR + self.vehicle.min_dist_to_check_collision):
                    center_dyn_obst = self.vehicle.shift_state(dyn_obst)
                    self.dyn_obstacle_segments.append(self.getBB(center_dyn_obst))
            
        self.current_state = new_state
        self.last_action = action
        center_state = self.vehicle.shift_state(new_state)
        observation, min_beam, lst_indexes = self.__getObservation(new_state)
        start_time = time.time()
        collision = self.isCollision(center_state, min_beam, lst_indexes)
        end_time = time.time()
        self.collision_time += (end_time - start_time)
        distanceToGoal = self.__goalDist(new_state)
        info["EuclideanDistance"] = distanceToGoal

        if self.hard_constraints:
            goalReached = distanceToGoal < self.HARD_EPS and abs(
                normalizeAngle(new_state.theta - self.goal.theta)) < self.ANGLE_EPS and abs(
                new_state.v - self.goal.v) < self.SPEED_EPS and abs(normalizeAngle(new_state.steer - self.goal.steer)) < self.STEERING_EPS
        elif self.soft_constraints:
            goalReached = distanceToGoal < self.HARD_EPS
        else:
            goalReached = distanceToGoal < self.SOFT_EPS and abs(
                normalizeAngle(new_state.theta - self.goal.theta)) < self.ANGLE_EPS

        #if not self.hardGoalReached and distanceToGoal < self.SOFT_EPS:
        #    self.hardGoalReached = True
        #if self.hardGoalReached:
        #    if distanceToGoal > self.SOFT_EPS:
        #        info["SoftEps"] = False

        reward = self.__reward(self.old_state, new_state, goalReached, collision, overSpeeding, overSteering)

        if not (self.stepCounter % self.UPDATE_SPARSE):
            self.old_state = self.current_state
        
        self.stepCounter += 1
        
        #DEBUG
        collision = False


        if goalReached or collision:
            if self.unionTask:
                if goalReached:
                    #print("goad benug")                
                    if not self.first_goal_reached:
                        #print("DEBUG")
                        self.first_goal_reached = True
                        print("first goal was achieved")
                        #self.goal = State(13, -5.5, degToRad(90), 0, 0)
                        self.goal = self.second_goal
                        self.task = -1
                        goalReached = False
                        isDone = False
                    else:
                        isDone = True
                        if collision:
                            info["Collision"] = True
                else:
                    isDone = True
                    if collision:
                        info["Collision"] = True
            else:
                isDone = True
                if collision:
                    info["Collision"] = True
        
        #print("DEBUG:", end=" ") # DEBUG
        #if "Collision" in info: # DEBUG
        #    print(info["Collision"], end=" ") # DEBUG
        #print(isDone) # DEBUG
        return observation, reward, isDone, info


    def drawBB(self, state, ego=True, draw_arrow=True, color="-c"):
        a = self.getBB(state, ego=ego)
        plt.plot([a[(i + 1) % len(a)][0].x for i in range(len(a) + 1)], [a[(i + 1) % len(a)][0].y for i in range(len(a) + 1)], color)
        if draw_arrow:
            plt.arrow(state[0], state[1], 2 * math.cos(state[2]), 2 * math.sin(state[2]), head_width=0.5, color='magenta')

    def drawObstacles(self, vertices, color="-b"):
        a = vertices
        plt.plot([a[(i + 1) % len(a)][0].x for i in range(len(a) + 1)], [a[(i + 1) % len(a)][0].y for i in range(len(a) + 1)], color)
        # if draw_arrow:
        #     plt.arrow(state[0], state[1], 2 * math.cos(state[2]), 2 * math.sin(state[2]), head_width=0.5, color='magenta')

    def render(self, reward, figsize=(10, 8), save_image=True):
        fig, ax = plt.subplots(figsize=figsize)

        x_delta = self.MAX_DIST_LIDAR
        y_delta = self.MAX_DIST_LIDAR

        x_min = self.current_state.x - x_delta
        x_max = self.current_state.x + x_delta
        ax.set_xlim(x_min, x_max)

        y_min = self.current_state.y - y_delta
        y_max = self.current_state.y + y_delta
        ax.set_ylim(y_min, y_max)
        
        if len(self.obstacle_segments) > 0:
            for obstacle in self.obstacle_segments:
                self.drawObstacles(obstacle)

        for dyn_obst in self.dynamic_obstacles:
            center_dyn_obst = self.vehicle.shift_state(dyn_obst)
            agentBB = self.getBB(center_dyn_obst)
            self.drawObstacles(agentBB)
            plt.arrow(dyn_obst.x, dyn_obst.y, 2 * math.cos(dyn_obst.theta), 2 * math.sin(dyn_obst.theta), head_width=0.5, color='magenta')
        
        ax.plot([self.current_state.x, self.goal.x], [self.current_state.y, self.goal.y], '--r')

        center_state = self.vehicle.shift_state(self.current_state)
        agentBB = self.getBB(center_state)
        self.drawObstacles(agentBB, color="-g")

        center_goal_state = self.vehicle.shift_state(self.goal)
        agentBB = self.getBB(center_goal_state)
        self.drawObstacles(agentBB, color="-g")

        vehicle_heading = Vec2d(cos(center_state.theta),
                 sin(center_state.theta)) * self.vehicle.length / 2
        ax.arrow(self.current_state.x, self.current_state.y,
                 vehicle_heading.x, vehicle_heading.y, width=0.1, head_width=0.3,
                 color='red')

        goal_heading = Vec2d(cos(center_goal_state.theta),
             sin(center_goal_state.theta)) * self.vehicle.length / 2
        ax.arrow(self.goal.x, self.goal.y, goal_heading.x,
                 goal_heading.y, width=0.1, head_width=0.3, color='cyan')

        for angle in self.angle_space:
            position = Vec2d(self.current_state.x, self.current_state.y)
            heading = Vec2d(cos(self.current_state.theta), sin(self.current_state.theta))
            heading = Ray(position, heading).rotate(angle).heading * self.__sendBeam(self.current_state, angle)

            ax.arrow(position.x, position.y, heading.x, heading.y, color='yellow')

        dx = self.goal.x - self.current_state.x
        dy = self.goal.y - self.current_state.y
        theta = radToDeg(self.current_state.theta)
        v = self.current_state.v
        delta = radToDeg(self.current_state.steer)
        Eps = self.vehicle.Eps
        v_s = self.vehicle.v_s
        a = self.vehicle.a

        ax.set_title(
            f'$dx={dx:.1f}, \
            dy={dy:.1f}, E={Eps:.2f},  v_s={v_s:.2f}, \
            theta={theta:.0f}^\\circ, v={v:.2f} \, m/s, \
            steer={delta:.0f}^\\circ, a = {a:.2f}, m/s^2, reward={reward:.0f}.$')
     
        if save_image:
            fig.canvas.draw()  # draw the canvas, cache the renderer
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            #image = image.reshape(1600, 2000, 3)
            plt.close('all')
            return image
        else:
            plt.pause(0.1)
            plt.show()
            # plt.close('all')

    def close(self):
        pass


class ObsNormEnvironment(gym.ActionWrapper):
    def action(self, action):
        action = np.array(action)
        act_k = (self.action_space.high - self.action_space.low) / 2.
        act_b = (self.action_space.high + self.action_space.low) / 2.
        return act_k * action + act_b

    def reverse_action(self, action):
        action = np.array(action)
        act_k = (self.action_space.high - self.action_space.low) / 2.
        act_b = (self.action_space.high + self.action_space.low) / 2.
        return (action - act_b) / act_k

    def step(self, *args, **kwargs):
        return self.env.step(*args, **kwargs)

    def render(self, *args, **kwargs):
        return self.env.render(*args, **kwargs)

    def reset(self, *args, **kwargs):
        return self.env.reset(*args, **kwargs)
