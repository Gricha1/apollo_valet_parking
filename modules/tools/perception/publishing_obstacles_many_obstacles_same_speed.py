#!/usr/bin/env python3

from cyber.python.cyber_py3 import cyber, cyber_time

import math
import time
import os
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

#from modules.localization.proto.localization_pb2 import LocalizationEstimate 

#from modules.dreamview.proto.hmi_config import HMIAction
"""
vehicle_pos_x = 587028.16
vehicle_pos_y = 4141491.93
vehicle_pos_z = 0

def callback(data):
    global vehicle_pos_x
    global vehicle_pos_y
    global vehicle_pos_z
    
    vehicle_pos_x = data.pose.position.x
    vehicle_pos_y = data.pose.position.y
    vehicle_pos_z = data.pose.position.z

"""
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
    
    map_type = "Highway4"  # NKB_cutted
    
    #Borregas Ave
    if map_type == 'Borregas_ave' :
    	routing_1_x = 587034.795623779
    	routing_1_y = 4141516.708694458
    	routing_2_x = 586952.61 
    	routing_2_y = 4141207.97
    	
    elif map_type == 'test' :  #NKB cutted with 3 lines 
    	routing_1_x = 395800.30
    	routing_1_y = 6246085.90
    	routing_2_x = 395979.71
    	routing_2_y = 6246081.28
    	
    elif map_type == 'Sunnyvale_Big_loop' :  #Sunnyvale Big loop
    	routing_1_x = 586869.96
    	routing_1_y = 4140260.31
    	routing_2_x = 587216.36
    	routing_2_y = 4140363.5
    
    elif map_type == 'NKB_cutted' :  #NKB_straight_line
    	routing_1_x = 395960.93
    	routing_1_y = 6246150.59
    	routing_2_x = 395314.47
    	routing_2_y = 6246161.79
    	
    elif map_type == 'Highway3' :  #highway3
    	routing_1_x = 420123.70
    	routing_1_y = 6182599.60
    	routing_2_x = 420489.18
    	routing_2_y = 6182618.67
    	
    elif map_type == 'Highway4' :  #highway4 long one
    	routing_1_x = 444084.79
    	routing_1_y = 6185607.61
    	routing_2_x = 444986.63   #first lane: 444996.08,second lane:444986.63 , third lane:
    	routing_2_y = 6185779.9 # first lane: 6185778.55,second lane:6185779.9, third lane: 	
    	
    else:
    	print ('map type does not match any map..')
    	routing_1_x = 587034.795623779
    	routing_1_y = 4141516.708694458
    	routing_2_x = 586952.61 
    	routing_2_y = 4141207.97

    starting_distance = 40 # starting distance between the 2 vehicles 
    
    
    apollo_test.send_routing_request( routing_1_x ,routing_1_y,routing_2_x,routing_2_y)
    
    #apollo_test.reader_node.spin()

   # time.sleep(1)
  #  start_x = vehicle_pos_x
    routing_2_x = 444996.08   #first lane: 444996.08,second lane:444986.63
    routing_2_y = 6185778.55 # first lane: 6185778.55,second lane:6185779.9
    
    theta = math.atan2(routing_2_y - routing_1_y,
                    routing_2_x - routing_1_x)
                    
    initial_x = routing_1_x + math.cos(theta) * starting_distance # initial position of obstacle
    initial_y = routing_1_y + math.sin(theta) * starting_distance # initial position of obstacle
    old_x= initial_x 
    old_y= initial_y
    
    initial_x3 = routing_1_x + math.cos(theta) * (starting_distance + 60 )# initial position of obstacle
    initial_y3 = routing_1_y + math.sin(theta) * (starting_distance + 60 )# initial position of obstacle
    old_x3= initial_x3
    old_y3= initial_y3
    delta_width = 3.4
    
    initial_x4 = routing_1_x + math.cos(theta) * (starting_distance +20 )# initial position of obstacle
    initial_y4 = routing_1_y + math.sin(theta) * (starting_distance +20 )# initial position of obstacle
    old_x4= initial_x4
    old_y4= initial_y4
    
    initial_x5 = routing_1_x + math.cos(theta) * (starting_distance +70 )# initial position of obstacle
    initial_y5 = routing_1_y + math.sin(theta) * (starting_distance +70 )# initial position of obstacle
    old_x5= initial_x5
    old_y5= initial_y5
    
    initial_x6 = routing_1_x + math.cos(theta) * (starting_distance +40 )# initial position of obstacle
    initial_y6 = routing_1_y + math.sin(theta) * (starting_distance +40 )# initial position of obstacle
    old_x6= initial_x6
    old_y6= initial_y6
    
    speed = 4 #3.8 #6  # m/s
    T = 0.1   #in seconds/ sleep time
    delta_s = T * speed
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
        
        """
        polygon_point_0 = obstacle.polygon_point.add()
        polygon_point_0.x = start_x + 5
        polygon_point_0.y = start_y + 16
        polygon_point_0.z = 0
      
        polygon_point_2 = obstacle.polygon_point.add()
        polygon_point_2.x = start_x + 5
        polygon_point_2.y = start_y + 12
        polygon_point_2.z = 0
        
        polygon_point_3 = obstacle.polygon_point.add()
        polygon_point_3.x = start_x + 3
        polygon_point_3.y = start_y + 12
        polygon_point_3.z = 0
        
        polygon_point_1 = obstacle.polygon_point.add()
        polygon_point_1.x = start_x + 3
        polygon_point_1.y = start_y + 16
        polygon_point_1.z = 0
        """
        obstacle.tracking_time = cyber_time.Time.now().to_sec() - start_time
        
        obstacle.type = 5
        obstacle.timestamp = time.time()
        
        
        
        obstacle3 = msg.perception_obstacle.add()
        obstacle3.id = 3
        obstacle3.theta = theta   # in radian
        obstacle3.position.x = old_x3 + math.cos(theta) * delta_s - math.sin(theta) * delta_width
        obstacle3.position.y = old_y3 + math.sin(theta) * delta_s + math.cos(theta) * delta_width
        obstacle3.position.z = 0
        old_x3 = old_x3 + math.cos(theta) * delta_s 
        old_y3 = old_y3 + math.sin(theta) * delta_s
        
        obstacle3.velocity.x = math.cos(theta) * speed
        obstacle3.velocity.y = math.sin(theta) * speed
        obstacle3.velocity.z = 0

        obstacle3.length = 4.565
        obstacle3.width = 2.082
        obstacle3.height = 1.35
        obstacle3.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle3.type = 5
        obstacle3.timestamp = time.time()
        
        
        
        obstacle4 = msg.perception_obstacle.add()
        obstacle4.id = 4
        obstacle4.theta = theta   # in radian
        obstacle4.position.x = old_x4 + math.cos(theta) * delta_s - math.sin(theta) * 2 * delta_width
        obstacle4.position.y = old_y4 + math.sin(theta) * delta_s + math.cos(theta) * 2 * delta_width
        obstacle4.position.z = 0
        old_x4 = old_x4 + math.cos(theta) * delta_s 
        old_y4 = old_y4 + math.sin(theta) * delta_s
        
        obstacle4.velocity.x = math.cos(theta) * speed
        obstacle4.velocity.y = math.sin(theta) * speed
        obstacle4.velocity.z = 0

        obstacle4.length = 4.565
        obstacle4.width = 2.082
        obstacle4.height = 1.35
        obstacle4.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle4.type = 5
        obstacle4.timestamp = time.time()
        
        
        obstacle5 = msg.perception_obstacle.add()
        obstacle5.id = 5
        obstacle5.theta = theta   # in radian
        obstacle5.position.x = old_x5 + math.cos(theta) * delta_s - math.sin(theta) * 4 * delta_width
        obstacle5.position.y = old_y5 + math.sin(theta) * delta_s + math.cos(theta) * 4 * delta_width
        obstacle5.position.z = 0
        old_x5 = old_x5 + math.cos(theta) * delta_s 
        old_y5 = old_y5 + math.sin(theta) * delta_s
        
        obstacle5.velocity.x = math.cos(theta) * speed
        obstacle5.velocity.y = math.sin(theta) * speed
        obstacle5.velocity.z = 0

        obstacle5.length = 4.565
        obstacle5.width = 2.082
        obstacle5.height = 1.35
        obstacle5.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle5.type = 5
        obstacle5.timestamp = time.time()
        
        
        obstacle6 = msg.perception_obstacle.add()
        obstacle6.id = 6
        obstacle5.theta = theta   # in radian
        obstacle6.position.x = old_x6 + math.cos(theta) * delta_s - math.sin(theta) * 6 * delta_width
        obstacle6.position.y = old_y6 + math.sin(theta) * delta_s + math.cos(theta) * 6 * delta_width
        obstacle6.position.z = 0
        old_x6 = old_x6 + math.cos(theta) * delta_s 
        old_y6 = old_y6 + math.sin(theta) * delta_s
        
        obstacle6.velocity.x = math.cos(theta) * speed
        obstacle6.velocity.y = math.sin(theta) * speed
        obstacle6.velocity.z = 0

        obstacle6.length = 4.565
        obstacle6.width = 2.082
        obstacle6.height = 1.35
        obstacle6.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle6.type = 5
        obstacle6.timestamp = time.time()
        
        """
        ### static obstacle
        obstacle2 = msg.perception_obstacle.add()
        obstacle2.id = 4
        obstacle2.theta = theta   # in radian
        obstacle2.position.x = initial_x - 3
        obstacle2.position.y = initial_y - 50
        obstacle2.position.z = 0
        obstacle2.velocity.x = 0
        obstacle2.velocity.y = 0
        obstacle2.velocity.z = 0
        obstacle2.length = 4.565
        obstacle2.width = 2.082
        obstacle2.height = 1.35
        obstacle2.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle2.type = 5
        obstacle2.timestamp = time.time()
        ### static obstacle
        """
        
        time.sleep(T)
        apollo_test.obstacle_writer.write(msg)        
"""
   # apollo_test.send_routing_request( 596573.0,4137634.0,596562.0,4137644.0)
    time.sleep(2.0)
    apollo_test.send_routing_request( 587034.795623779,4141516.708694458,586980.0,4141311.0) # Borregas Ave/ overtaking scenario
    # before intersaction 586980.0,4141311.0
    # after intersaction 586952.0,4141210.0
    print('A routing request has been sent ..')
   
   
"""
   
   
   
   
   
   
   
   
