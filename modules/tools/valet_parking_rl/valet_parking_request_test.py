#!/usr/bin/env python3
from ast import arg
import sys
import math
import time
import os
import json
import matplotlib.pyplot as plt
import numpy as np
from cyber.python.cyber_py3 import cyber, cyber_time
import argparse
import http.client
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

sys.path.insert(0, "modules/tools/valet_parking_rl/POLAMP_sample_factory_/")

parser = argparse.ArgumentParser(description='Get max steps for validation')
parser.add_argument('-map', '--map')
parser.add_argument('-parking_place', '--parking_place', type=int)
args = parser.parse_args()

class point:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		
class ApolloValetParkingRequestInterface:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request',
		 RoutingRequest)
     
    def launch_control(self):
        os.system('cyber_launch start\
			 /apollo/modules/control/launch/control.launch &')
    
    def launch_routing(self):
        os.system('cyber_launch start\
		 /apollo/modules/routing/launch/routing.launch &')
    
    def launch_perception(self):
        os.system('cyber_launch start\
		 /apollo/modules/transform/launch/static_transform.launch &')

        os.system('cyber_launch start\
		 /apollo/modules/perception/production/launch/perception.launch &')
    
    def launch_prediction(self):
        os.system('cyber_launch start\
		 /apollo/modules/prediction/launch/prediction.launch &')
    
    def launch_rtk_localization(self):
        os.system('ldconfig -p | grep libcuda.so')
        os.system('cyber_launch start \
		/apollo/modules/localization/launch/rtk_localization.launch &')
    
    def launch_planning(self):
        os.system('cyber_launch start \
		/apollo/modules/planning/launch/planning.launch &')
    
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
			
if __name__ == "__main__":
	apollo_valet_parking_interface = ApolloValetParkingRequestInterface()
	second_req_sent = False
	parking_req_sent = False
	start_time = cyber_time.Time.now().to_sec()
	while not cyber.is_shutdown():
		tracking_time = cyber_time.Time.now().to_sec() - start_time
		if (tracking_time > 2 and not parking_req_sent):
			parking_req_sent = True
			parking_msg = RoutingRequest()
			parking_msg.header.module_name = 'dreamview'
			parking_msg.header.sequence_num = 0
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
				elif args.parking_place == 5: #not working
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
			elif args.map == "parking_lot":
				#паркинг лот с одним местом
				x_start = 389049.52
				y_start = 221176.54
				number_of_place = args.number_of_place
				#number_of_place = 82
				if number_of_place == 82:	
					x_end = 389008
					y_end = 221155
					center_parkin_place_x = 388999.21
					center_parkin_place_y = 221150.98
				elif number_of_place == 76:
					x_end = 389008
					y_end = 221155
					center_parkin_place_x = 388983.08
					center_parkin_place_y = 221151.00
				elif number_of_place == 91:
					x_end = 389032.75
					y_end = 221155
					center_parkin_place_x = 389020.83
					center_parkin_place_y = 221150.75
			else:
				assert 1 == 0, "please specify -map parameter: \
						-map test, -map parking_lot"
			waypoint1 = parking_msg.waypoint.add()
			waypoint1.pose.x = float(x_start)
			waypoint1.pose.y = float(y_start)
			waypoint1 = parking_msg.waypoint.add()
			waypoint1.pose.x = float(x_end)
			waypoint1.pose.y = float(y_end)
			parking_msg.parking_info.parking_point.x = float(center_parkin_place_x)
			parking_msg.parking_info.parking_point.y = float(center_parkin_place_y)
			parking_msg.parking_info.parking_space_id = f"place_{number_of_place}"
			apollo_valet_parking_interface.routing_writer.write(parking_msg)
			print('PARKING REQUEST')
			print('parking map:', args.map)
			print('parking place:', number_of_place)




				