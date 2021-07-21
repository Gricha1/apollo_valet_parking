#!/usr/bin/env python3

from cyber.python.cyber_py3 import cyber, cyber_time

import math
import time
import os
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

#from modules.localization.proto.localization_pb2 import LocalizationEstimate 

#from modules.dreamview.proto.hmi_config import HMIAction

class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
        
      #  self.reader_node = cyber.Node("reader")
      #  self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)
        

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

        

if __name__ == '__main__':
    apollo_test = ApolloFeatures()
    seq=0
    time.sleep(2.0)
    
    map_type = "test_scene"  # NKB_cutted
    
    #Borregas Ave
    if map_type == 'test_scene' :
    	routing_1_x = 388142.04
    	routing_1_y = 220997.57
    	routing_2_x = 388882.00
    	routing_2_y = 221082.19
    	#routing_2_x = 388969.65 
    	#routing_2_y = 221207.59


    starting_distance = 40 # starting distance between the 2 vehicles 
    
    
    apollo_test.send_routing_request( routing_1_x ,routing_1_y,routing_2_x,routing_2_y)
    print('First routing request has been sent ..')
    #apollo_test.reader_node.spin()

   # time.sleep(1)
  #  start_x = vehicle_pos_x
    routing_1_x = 388227.42    #first lane:388227.42
    routing_1_y = 221082.52    #first lane:221082.87 
    routing_2_x = 389336.23   #first lane:389336.23   ,second lane:
    routing_2_y = 221082.01 # first lane: 221086.19,second lane:
    
    theta = math.atan2(routing_2_y - routing_1_y,
                    routing_2_x - routing_1_x)
                    
    initial_x = routing_1_x + math.cos(theta) * starting_distance # initial position of obstacle
    initial_y = routing_1_y + math.sin(theta) * starting_distance # initial position of obstacle
    old_x= initial_x 
    old_y= initial_y
    
    initial_x3 = routing_1_x + math.cos(theta) * (starting_distance - 60 )# initial position of obstacle
    initial_y3 = routing_1_y + math.sin(theta) * (starting_distance - 60 )# initial position of obstacle
    old_x3= initial_x3
    old_y3= initial_y3
    delta_width = 3.4
    
    
    second_req_sent = False
    parking_req_sent = False
    speed = 3.8 #3.8 #6  # m/s
    speed3 = 8
    T = 0.1   #in seconds/ sleep time
    delta_s = T * speed
    delta_s3 = T* speed3
    start_time = cyber_time.Time.now().to_sec()
    while not cyber.is_shutdown():
     #   print('publishing obstacles')
        msg = PerceptionObstacles()
        msg.header.module_name = 'perception_obstacle'
        msg.header.sequence_num = seq
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
        seq= seq+1
        obstacle = msg.perception_obstacle.add()
        
        obstacle.id = 2
        obstacle.theta = theta   # in radian
        obstacle.position.x = old_x + math.cos(theta) * delta_s
        obstacle.position.y = old_y + math.sin(theta) * delta_s
        obstacle.position.z = 0
        old_x = obstacle.position.x 
        old_y = obstacle.position.y
        
        obstacle.velocity.x = math.cos(theta) * speed
        obstacle.velocity.y = math.sin(theta) * speed
        obstacle.velocity.z = 0

        obstacle.length = 4.565
        obstacle.width = 2.082
        obstacle.height = 1.35
        
        tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle.tracking_time = tracking_time
        
        obstacle.type = 5
        obstacle.timestamp = time.time()
   

        
        time.sleep(T)
        apollo_test.obstacle_writer.write(msg)
       # print(tracking_time)
        if (tracking_time > 86 and not second_req_sent ):    # seconds
            second_req_sent = True
            routing_msg = RoutingRequest()
            routing_msg.header.module_name = 'dreamview'
            routing_msg.header.sequence_num = 1
            waypoint = routing_msg.waypoint.add()
            waypoint.pose.x = float(388706.52)
            waypoint.pose.y = float(221082.37)
            routing_x = 388969.65 
            routing_y = 221207.59
            waypoint = routing_msg.waypoint.add()
            waypoint.pose.x = float(routing_x)
            waypoint.pose.y = float(routing_y)
            apollo_test.routing_writer.write(routing_msg)
            time.sleep(T)
            print('Second routing request has been sent ..')
        if (tracking_time > 140 and not parking_req_sent):
            parking_req_sent = True
            parking_msg = RoutingRequest()
            parking_msg.header.module_name = 'dreamview'
            parking_msg.header.sequence_num = 0

            x_start =  388969.65
            y_start = 221207.59

            x_end = 388989.25
            y_end = 221207.91

            waypoint1 = parking_msg.waypoint.add()
            waypoint1.pose.x = float(x_start)
            waypoint1.pose.y = float(y_start)
            waypoint1 = parking_msg.waypoint.add()
            waypoint1.pose.x = float(x_end)
            waypoint1.pose.y = float(y_end)
            parking_msg.parking_info.parking_point.x = float(388979.02)
            parking_msg.parking_info.parking_point.y = float(221199.04)
            parking_msg.parking_info.parking_space_id = "parking_space_1"
            apollo_test.routing_writer.write(parking_msg)
            
            time.sleep(T)
            parking_msg = RoutingRequest()
            parking_msg.header.module_name = 'dreamview'
            parking_msg.header.sequence_num = 0
            waypoint1 = parking_msg.waypoint.add()
            waypoint1.pose.x = float(x_start)
            waypoint1.pose.y = float(y_start)
            waypoint1 = parking_msg.waypoint.add()
            waypoint1.pose.x = float(x_end)
            waypoint1.pose.y = float(y_end)
            parking_msg.parking_info.parking_point.x = float(388979.02)
            parking_msg.parking_info.parking_point.y = float(221199.04)
            parking_msg.parking_info.parking_space_id = "parking_space_1"
            apollo_test.routing_writer.write(parking_msg)
            print('parking request has been sent ..')


   
   
   
   
   
   
   
   
