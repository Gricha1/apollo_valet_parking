#!/usr/bin/env python3

from cyber.python.cyber_py3 import cyber, cyber_time

import math
import time
import os
import argparse
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.planning.proto.planning_pb2 import roi_boundary_message

parser = argparse.ArgumentParser(description='Get max steps for validation')
parser.add_argument('-map', '--map')
parser.add_argument('-parking_place', '--parking_place', type=int)
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

class ApolloValetParkingRequestInterface:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("obst_apollo_interface")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', 
                                                    RoutingRequest)
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', 
                                                PerceptionObstacles)
    
    def launch_control(self):
        os.system('cyber_launch start /apollo/modules/control/launch/control.launch &')
    
    def launch_routing(self):
        os.system('cyber_launch start /apollo/modules/routing/launch/routing.launch &')
    
    def launch_perception(self):
        os.system('cyber_launch start /apollo/modules/transform/launch/static_transform.launch &')
        os.system('cyber_launch start /apollo/modules/perception/production/launch/perception.launch &')
    
    def launch_prediction(self):
        os.system('cyber_launch start /apollo/modules/prediction/launch/prediction.launch &')
    
    def launch_rtk_localization(self):
        os.system('ldconfig -p | grep libcuda.so')
        os.system('cyber_launch start /apollo/modules/localization/launch/rtk_localization.launch &')
    
    def launch_planning(self):
        os.system('cyber_launch start /apollo/modules/planning/launch/planning.launch &')
    
    def launch_all_modules(self):
        self.launch_rtk_localization()
        self.launch_perception()
        self.launch_prediction()
        self.launch_planning()
        self.launch_control()
        self.launch_routing()
    
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

def setDynamicPositionAndMsgInfo(obstacle, 
                        old_x, old_y, theta, step_distance):
    obstacle.id = 1
    obstacle.theta = theta # radian
    obstacle.position.x = old_x + math.cos(theta) * step_distance
    obstacle.position.y = old_y + math.sin(theta) * step_distance
    obstacle.position.z = 0
    obstacle.velocity.x = math.cos(theta) * first_dyn_speed
    obstacle.velocity.y = math.sin(theta) * first_dyn_speed
    obstacle.velocity.z = 0
    obstacle.length = 4.565
    obstacle.width = 2.082
    obstacle.height = 1.35
    tracking_time = cyber_time.Time.now().to_sec() - start_time
    obstacle.tracking_time = tracking_time
    obstacle.type = 5
    obstacle.timestamp = time.time()
    
if __name__ == '__main__':
    apollo_valet_parking_request_interface = ApolloValetParkingRequestInterface()
    apollo_stage_manager_interface = ApolloStageManagerInterface()
    apollo_rl_interface = ApolloRLInterface()
    if args.map == "test": # тестовые 10 парковочных мест
        x_start = 388929.22559400817
        y_start = 221208.13388718164
        x_end = 389028.85093914089
        y_end = 221208.13388718164
        if args.parking_place == 1:
            center_parkin_place_x = 388959.08
            center_parkin_place_y = 221201.40
            number_of_place = 1
        elif args.parking_place == 2:
            center_parkin_place_x = 388964.21
            center_parkin_place_y = 221201.40
            number_of_place = 2
        elif args.parking_place == 3:
            center_parkin_place_x = 388969.19
            center_parkin_place_y = 221201.40
            number_of_place = 3
        elif args.parking_place == 5:
            center_parkin_place_x = 388979.21
            center_parkin_place_y = 221201.40
            number_of_place = 5
        elif args.parking_place == 6:
            center_parkin_place_x = 388984.21
            center_parkin_place_y = 221201.40
            number_of_place = 6
        elif args.parking_place == 7:
            center_parkin_place_x = 388989.21
            center_parkin_place_y = 221201.40
            number_of_place = 7
    if args.test_case == 1:
        pass
    if args.test_case == 2:
        pass
    if args.test_case == 3:
        pass
    if args.test_case == 4:
        pass
    elif args.test_case == 5:
        current_dyn_x = center_parkin_place_x + 14
        current_dyn_y = center_parkin_place_y + 7.8
        current_dyn_theta = math.pi # radians
        first_dyn_speed = 0.5 # m/s
    elif args.test_case == 6:
        current_dyn_x = center_parkin_place_x + 14
        current_dyn_y = center_parkin_place_y + 4.5
        current_dyn_theta = math.pi # radians
        first_dyn_speed = 0.5 # m/s
    else:
        current_dyn_x = 388998.19
        current_dyn_y = 221211.71
        current_dyn_theta = math.pi # radians
        first_dyn_speed = 0.5 # m/s
    time_step = 0.1
    step_distance = time_step * first_dyn_speed
    start_time = cyber_time.Time.now().to_sec()
    seq = 0
    while not cyber.is_shutdown():
        if apollo_stage_manager_interface.is_parking_approuch_end:
            msg = PerceptionObstacles()
            msg.header.module_name = 'perception_obstacle'
            msg.header.sequence_num = seq
            msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
            seq = seq + 1
            dyn_obstacle = msg.perception_obstacle.add()
            setDynamicPositionAndMsgInfo(dyn_obstacle, 
                    current_dyn_x, current_dyn_y, 
                    current_dyn_theta, step_distance)
            current_dyn_x = dyn_obstacle.position.x 
            current_dyn_y = dyn_obstacle.position.y
            tracking_time = dyn_obstacle.tracking_time
            time.sleep(time_step)
            apollo_valet_parking_request_interface.obstacle_writer.write(msg)




   
   
   
   
   
   
