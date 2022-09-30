#!/usr/bin/env python3
from cyber.python.cyber_py3 import cyber, cyber_time
import sys
import math
import time
import os
import argparse
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.planning.proto.planning_pb2 import roi_boundary_message
from modules.localization.proto.localization_pb2 import LocalizationEstimate
sys.path.insert(0, "modules/tools/valet_parking_rl/POLAMP_sample_factory_/")
from rl_utils.utils import State, Vehicle, normalizeFromZeroTo2Pi
#from dataset_generation.utlis import getTestTasks
from dataset_generation.apollo_dataset_utils import getApolloTestTasks
import json

with open("modules/tools/valet_parking_rl/POLAMP_sample_factory_/configs/car_configs.json", 'r') as f:
    car_config = json.load(f)

parser = argparse.ArgumentParser(description='Get max steps for validation')
parser.add_argument('-test_case', '--test_case', type=int)
args = parser.parse_args()

class ApolloRLInterface:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("rl_trajectory_check_interface")
        self.reader = self.node.create_reader('from_python_to_apollo', 
                                    roi_boundary_message, self.callback)
        self.is_rl_trajectory_ready = False

    def callback(self, data):
        self.is_rl_trajectory_ready = True

class ApolloStageManagerInterface:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("stage_check_interface")
        self.reader = self.node.create_reader('get_roi_boundaries_topic', 
                                    roi_boundary_message, self.callback)
        self.is_parking_approuch_end = False

    def callback(self, data):
        self.is_parking_approuch_end = True

class EgoCarPositionReader:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("dynamic_publisher_ego_car_listener")
        self.reader = self.node.create_reader(
                                '/apollo/localization/pose', 
                                LocalizationEstimate, 
                                self.callback)
        self.ego_pose = None

    def callback(self, data):
        self.ego_pose = [data.pose.position.x, data.pose.position.y] 

    def get_ego_pose(self):

        return self.ego_pose
        
class ApolloValetParkingRequestInterface:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("obst_apollo_interface")
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', 
                                                PerceptionObstacles)

    def send_routing_request(self, x_start, y_start, x_end, y_end):
        msg = RoutingRequest()
        msg.header.module_name = 'dreamview'
        msg.header.sequence_num = 0
        waypoint = msg.waypoint.add()
        waypoint.pose.x = float(x_start)
        waypoint.pose.y = float(y_start)
        waypoint = msg.waypoint.add()
        waypoint.pose.x = float(x_end)
        waypoint.pose.y = float(y_end)
        time.sleep(2.0)
        self.routing_writer.write(msg)

def transform_dynamic_state_to_apollo_state(gym_dynamic_state,
                                            gym_ego_start_state,
                                            apollo_ego_start_state):
    gym_dynamic_x, gym_dynamic_y = gym_dynamic_state.x, gym_dynamic_state.y
    gym_ego_x, gym_ego_y = gym_ego_start_state[0], gym_ego_start_state[1]
    apollo_ego_x, apollo_ego_y = apollo_ego_start_state

    dynamic_dx = gym_dynamic_x - gym_ego_x
    dynamic_dy = gym_dynamic_y - gym_ego_y

    apollo_dynamic_x = apollo_ego_x + dynamic_dx
    apollo_dynamic_y = apollo_ego_y + dynamic_dy
    apollo_dynamic_theta = normalizeFromZeroTo2Pi(gym_dynamic_state.theta)
    apollo_dynamic_v = gym_dynamic_state.v

    return apollo_dynamic_x, apollo_dynamic_y, \
           apollo_dynamic_theta, \
           apollo_dynamic_v

def setDynamicPositionAndMsgInfo(
                                gym_dynamic,
                                gym_ego_start_state,
                                apollo_ego_start_state,
                                apollo_dynamic_obstacle,
                                start_time
                                ):

    action = gym_dynamic.get_action()
    gym_dynamic.step(action=action)
    gym_dynamic_state = gym_vehicle.getCurrentState()

    new_gym_dynamic_state = transform_dynamic_state_to_apollo_state(
                                            gym_dynamic_state, 
                                            gym_ego_start_state,
                                            apollo_ego_start_state)

    new_dynamic_x, new_dynamic_y = new_gym_dynamic_state[0], new_gym_dynamic_state[1]
    new_dynamic_theta, new_dynamic_v = new_gym_dynamic_state[2], new_gym_dynamic_state[3]

    apollo_dynamic_obstacle.id = 1
    apollo_dynamic_obstacle.theta = new_dynamic_theta # radian

    apollo_dynamic_obstacle.position.x = new_dynamic_x
    apollo_dynamic_obstacle.position.y = new_dynamic_y
    apollo_dynamic_obstacle.position.z = 0

    apollo_dynamic_obstacle.velocity.x = math.cos(new_dynamic_theta) * new_dynamic_v
    apollo_dynamic_obstacle.velocity.y = math.sin(new_dynamic_theta) * new_dynamic_v
    apollo_dynamic_obstacle.velocity.z = 0
    
    apollo_dynamic_obstacle.length = gym_dynamic_state.length
    apollo_dynamic_obstacle.width = gym_dynamic_state.width
    apollo_dynamic_obstacle.height = 1.35
    tracking_time = cyber_time.Time.now().to_sec() - start_time
    apollo_dynamic_obstacle.tracking_time = tracking_time
    apollo_dynamic_obstacle.type = 5
    apollo_dynamic_obstacle.timestamp = time.time()
    
if __name__ == '__main__':
    apollo_valet_parking_request_interface = ApolloValetParkingRequestInterface()
    apollo_stage_manager_interface = ApolloStageManagerInterface()
    apollo_ego_car_reader = EgoCarPositionReader()
    
    # test
    #test_tasks = getTestTasks(car_config)
    test_tasks = getApolloTestTasks(car_config)

    current_task_in_gym_env = test_tasks[args.test_case - 1]
    _, gym_ego_start_goal_position, dynamic_dym_obstacles = current_task_in_gym_env
    gym_ego_start, _ = gym_ego_start_goal_position
    assert len(gym_ego_start) == 5, "gym ego start must have len 5" \
        + f"but {len(gym_ego_start)} given and gym ego start is {gym_ego_start}"

    dyn_gym_obst = dynamic_dym_obstacles[0]
    gym_vehicle = Vehicle(car_config, ego_car=False)
    state = State(dyn_gym_obst[0],
                    dyn_gym_obst[1],
                    dyn_gym_obst[2],
                    dyn_gym_obst[3],
                    dyn_gym_obst[4])
    gym_vehicle.reset(state, dyn_gym_obst[5])

    time_step = 0.1
    start_time = cyber_time.Time.now().to_sec()
    seq = 0
    is_apollo_ego_init_pose_read = False
    while not cyber.is_shutdown():
        if apollo_stage_manager_interface.is_parking_approuch_end:
            msg = PerceptionObstacles()
            msg.header.module_name = 'perception_obstacle'
            msg.header.sequence_num = seq
            msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
            seq = seq + 1
            apollo_dynamic_obstacle = msg.perception_obstacle.add()

            if not is_apollo_ego_init_pose_read:
                apollo_ego_start_position = apollo_ego_car_reader.get_ego_pose()
                assert not(apollo_ego_start_position is None), \
                    "ego car possition wasnt recieved from localization/pose topic"
                is_apollo_ego_init_pose_read = True

            setDynamicPositionAndMsgInfo(gym_vehicle,
                                         gym_ego_start,
                                         apollo_ego_start_position,
                                         apollo_dynamic_obstacle,
                                         start_time
                                         )
            time.sleep(time_step)
            apollo_valet_parking_request_interface.obstacle_writer.write(msg)




   
   
   
   
   
   
