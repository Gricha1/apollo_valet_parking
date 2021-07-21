#!/usr/bin/env python3

from cyber.python.cyber_py3 import cyber, cyber_time

import math
import time
import os
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint, ParkingInfo
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.map.proto.map_parking_space_pb2 import ParkingSpace, ParkingLot
from modules.common.proto.geometry_pb2 import PointENU

class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
        
      #  self.reader_node = cyber.Node("reader")
      #  self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)
        
  
    	
    
   # def send_routing_request(self):
        

        

if __name__ == '__main__':
    apollo_test = ApolloFeatures()

    time.sleep(2.0)
    
    
    msg = RoutingRequest()
    msg.header.module_name = 'dreamview'
    msg.header.sequence_num = 0
    #### san_mateo
    """    
    x_start = 559872.85
    y_start = 4157651.77
      
    x_end = 559890.47
    y_end = 4157631.75
    """
    #### test_scene
       
   # x_start = 388051.44
  #  y_start = 221152.32
      
   # x_end = 388024.90
   # y_end = 221152.27
    ## parking_space_1
    x_start = 388969.65
    y_start = 221207.59

    x_end = 388989.25
    y_end = 221207.91
    ## parking_space_2
   # x_start = 388989.25
   # y_start = 221207.91

   # x_end = 389012.47
   # y_end = 221207.59
    """
    #### sunnyvale 
    x_start =587164.89# 587191.89 
    y_start =4141435.74#4141422.39 
      
    x_end = 587215.07
    y_end = 4141416.96
    """
    #####
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_start)
    waypoint.pose.y = float(y_start)
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_end)
    waypoint.pose.y = float(y_end)
    
    msg.parking_info.parking_space_id = "parking_space_1"
    #### test_scene
     #upper parking 1
   # msg.parking_info.parking_point.x = float(388038.94)
   # msg.parking_info.parking_point.y = float(221159.11)
     #lower parking 2
    ##parking_space_1
    msg.parking_info.parking_point.x = float(388979.02)
    msg.parking_info.parking_point.y = float(221199.04)
    ##parking_space_2
  #  msg.parking_info.parking_point.x = float(388998.83)
  #  msg.parking_info.parking_point.y = float(221200.00)
    """
    ### sunnyvale_two_offices
    msg.parking_info.parking_point.x = float(587199.69)
    msg.parking_info.parking_point.y = float(4141415.4)
    """
    ###san mateo
    #msg.parking_info.parking_point.x = float(559881.53)
   # msg.parking_info.parking_point.y = float(4157634.07)
   # msg.parking_space.id.id = "pakring_space_1"
    """
    polygonpoint = msg.parking_space.polygon.point.add()
    polygonpoint.x = float(388038.6538108855)
    polygonpoint.y = float(221162.6057218537) 


    polygonpoint = msg.parking_space.polygon.point.add()
    polygonpoint.x = float(388041.24532293412)
    polygonpoint.y = float(221161.9138005795)


    polygonpoint = msg.parking_space.polygon.point.add()
    polygonpoint.x = float(388040.11132028769)
    polygonpoint.y = float(221156.9443679708)


    polygonpoint = msg.parking_space.polygon.point.add()
    polygonpoint.x = float(388037.50218934589)
    polygonpoint.y = float(221157.6283384813)
    """
  #  overlap_id = msg.parking_space.overlap_id.add()
  #  overlap_id = "overlap_2659"

  #  overlap_id = msg.parking_space.overlap_id.add()
  #  overlap_id = "overlap_2660"

  #  msg.parking_space.heading = float(1.345454)   

    apollo_test.routing_writer.write(msg)
   # time.sleep(117.0)
    """
    msg2 = RoutingRequest()
    msg2.header.module_name = 'dreamview'
    msg2.header.sequence_num = 1
    
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = 586846.69
    waypoint.pose.y = 4140789.55
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_2)
    waypoint.pose.y = float(y_2)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_3)
    waypoint.pose.y = float(y_3)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_4)
    waypoint.pose.y = float(y_4)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_5)
    waypoint.pose.y = float(y_5)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_6)
    waypoint.pose.y = float(y_6)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_7)
    waypoint.pose.y = float(y_7)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_8)
    waypoint.pose.y = float(y_8)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_9)
    waypoint.pose.y = float(y_9)
        
        
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_start)
    waypoint.pose.y = float(y_start)

   # time.sleep(2.0)
    apollo_test.routing_writer.write(msg2)
    """
   
    print('A routing request has been sent ..')
   
  
   
   
   
   
   
   
   
