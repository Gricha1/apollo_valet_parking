from pickle import FALSE
from modules.tools.custom_2.EnvLib.utils import *
from math import *

class CarLikeRobot:

    def __init__(self, config):
        self.max_v = config['max_v']
        self.max_acc = config['max_acc']
        self.max_steer = config['max_steer']
        self.max_omega = config['max_omega']
        self.wheel_base = config['wheel_base']

    def oneMoveStep(self, v_des, ang_vel_des, x, y, theta, v, angV, strAng, dt):
        if v_des > self.max_v:
            v_des = self.max_v
        elif v_des < -self.max_v:
            v_des = -self.max_v

        if (v_des - v)/dt  > self.max_acc:
            v_des = v + self.max_acc * dt
        elif (v_des - v)/dt  < -self.max_acc:
            v_des = v - self.max_acc * dt
        
        strAngDes = atan(ang_vel_des * self.wheel_base / v_des)

        if strAngDes > self.max_steer:
            strAngDes = self.max_steer
        elif strAngDes < -self.max_steer:
            strAngDes = -self.max_steer

        if (strAngDes - strAng) / dt > self.max_omega:
            strAngDes = strAng + self.max_omega * dt
        elif (strAngDes - strAng)/dt < -self.max_omega:
            strAngDes = strAng - self.max_omega * dt
        
        strAng = strAngDes
        radius = self.wheel_base / tan(strAng + 1e-6)
        x += v_des * cos(theta) * dt
        y += v_des * sin(theta) * dt
        theta += (v_des / radius) * dt
        v = v_des
        theta = normalizeAngle(theta)

        return x, y, theta, v, angV, strAng

K_rho = 1
K_alpha = 6
K_beta = -5

robot_config = {
    'max_v': kmToM(10),
    'max_acc': 5,
    'max_steer': pi/6,
    'max_omega': 1.0,
    'wheel_base': 2.5
}

def distOrient(orient1, orient2):
    while orient1 < 0:
        orient1 += 2 * pi

    while orient2 < 0:
        orient2 += 2 * pi

    return min(abs(orient1 - orient2), 2 * pi - abs(orient1 - orient2))



def validatePOSQ(valTasks, checkVelocity=False, toGoal=False):

    current_state = valTasks[0][0]
    goal = valTasks[0][1]
    robot = CarLikeRobot(robot_config)
    x, y, theta, v, steer = current_state
    angV = 0
    lst_params = []
    curr_state = [x, y, theta, v, angV, steer]
    
    for _ in range(300):
        x, y, theta, v, angV, steer = curr_state
        lst_params.append([x, y, theta, v, steer])
        dx = goal[0] - x
        dy = goal[1] - y
        rho = (dx ** 2 + dy ** 2) ** (1/2)
        # print(rho)
        
        # and abs(normalizeAngle(goal[2] - theta)) < pi/18
        if rho < 1.0:
            if not toGoal:
                return True, lst_params
            else:
                if rho < 0.5:
                    if abs(normalizeAngle(theta - goal[2])) < (math.pi / 18.):
                        return True, lst_params
                
        alpha = normalizeAngle(atan2(dy,dx) - theta)
        beta = normalizeAngle(goal[2] - theta)
        beta = normalizeAngle(beta - alpha)

        v_des = K_rho * rho
        angVDes = K_alpha * alpha + K_beta * beta
        curr_state = robot.oneMoveStep(v_des, angVDes, x, y, theta, v, angV, steer, 0.1)
    
    return False, []


def generateTaskToPOSQ():
    total = 0
    success_rate = 0
    start = [0. for _ in range(5)]
    goal = [0. for _ in range(5)]
    
    for s_theta in range(-10, 10 + 1):
        # print(f"s_theta {s_theta}")
        start[3] = s_theta * math.pi / 30
        for g_theta in range(-10, 10 + 1):
            goal[4] = g_theta * math.pi
            for x in range(5, 10):
                goal[0] = x
                total += 1
                success, _ = validatePOSQ([(start, goal)], toGoal=True)
                if(success):
                    success_rate += 1
    
    print(f"total {total}")
    return success_rate / total * 100.
    