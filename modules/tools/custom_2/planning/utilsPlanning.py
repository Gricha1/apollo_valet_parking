import numpy as np
import math
import matplotlib.pylab as plt
from modules.tools.custom_2.EnvLib.utils import *
from copy import deepcopy

def getBB(state, w = 2.0, l=3.8, ego=True, front_axis=True):
    new_state = deepcopy(state)
    x = new_state[0]
    y = new_state[1]
    angle = new_state[2]
    if ego:
        w = w / 2
        l = l / 2
        if front_axis:
            rear_to_center = 0.65
            shift = l - rear_to_center
            toCenter = True
            shift = -shift if toCenter else shift
            x = x + shift * math.cos(angle)
            y = y + shift * math.sin(angle)
    else:
        w = new_state[3]
        l = new_state[4]
    BBPoints = [(-l, -w), (l, -w), (l, w), (-l, w)]
    vertices = []
    sinAngle = math.sin(angle)
    cosAngle = math.cos(angle)
    for i in range(len(BBPoints)):
        new_x = cosAngle * (BBPoints[i][0]) - sinAngle * (BBPoints[i][1])
        new_y = sinAngle * (BBPoints[i][0]) + cosAngle * (BBPoints[i][1])
        vertices.append([new_x + x, new_y + y])
    
    return vertices

def drawBB(state, ego=True, draw_arrow=True, color="-c", front_axis=True, color_arrow='magenta'):
    a = getBB(state, ego=ego, front_axis=front_axis)
    plt.plot([a[(i + 1) % len(a)][0] for i in range(len(a) + 1)], [a[(i + 1) % len(a)][1] for i in range(len(a) + 1)], color)
    if draw_arrow:
        plt.arrow(state[0], state[1], 2 * math.cos(state[2]), 2 * math.sin(state[2]), head_width=0.5, color=color_arrow)

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

        return [x_start, y_start, theta_start], [new_x_goal, new_y_goal, normalizeAngle(theta_goal)]
    
    def rotateState(self, state):
        sx = state[0]
        sy = state[1]
        stheta = state[2]
        sx -= self.diff_x
        sy -= self.diff_y
        stheta -= self.theta
        new_sx = self.cos_theta * sx + self.sin_theta * sy
        new_sy = - self.sin_theta * sx + self.cos_theta * sy
        
        return [new_sx, new_sy, normalizeAngle(stheta)]

    def inverseRotate(self, state):
        x = state[0]
        y = state[1]
        theta = state[2]
        new_x = self.cos_theta * x - self.sin_theta * y
        new_y = self.sin_theta * x + self.cos_theta * y
        new_x += self.diff_x
        new_y += self.diff_y
        theta += self.theta
        return new_x, new_y, normalizeAngle(theta)
