#!/usr/bin/env python3
import sys
import math
import time
import os
import json
import matplotlib.pyplot as plt
import numpy as np
from cyber.python.cyber_py3 import cyber, cyber_time
from modules.tools.custom_2.validateModel import *
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.planning.proto.planning_pb2 import roi_boundary_message
import argparse


parser = argparse.ArgumentParser(description='Get max steps for validation')
parser.add_argument('-s', '--max_steps', type=int, 
                    help='max_steps = steps in env')
args = parser.parse_args()



class point:

	def __init__(self, x, y):

		self.x = x
		self.y = y
		

class ApolloFeatures:

    def __init__(self):

        cyber.init()
        self.node = cyber.Node("apollo_features")

        self.routing_writer = self.node.create_writer('/apollo/routing_request',
		 RoutingRequest)

        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles',
		 PerceptionObstacles)
        

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


class RoiWriter:

    def __init__(self):

        cyber.init()
        self.node = cyber.Node("roi_writer")

        self.trajectory_writer = self.node.create_writer('from_python_to_apollo', 
														roi_boundary_message)

'''
	def send_trajectory(self, trajectory):
        msg = RoutingRequest()
        msg.header.module_name = 'dreamview'
        msg.header.sequence_num = 0

		for state in trajectory:
			waypoint = msg.waypoint.add()
			waypoint.pose.x = float(state.x)
			waypoint.pose.y = float(state.y)

        self.trajectory_writer.write(msg)
'''


class RoiReader:

	def __init__(self, polamp_trajectory_writer):
		cyber.init()
		self.node = cyber.Node("listener")
		self.reader = self.node.create_reader('get_roi_boundaries_topic', 
										roi_boundary_message, self.callback)
		self.j = 0

		self.polamp_trajectory_writer = polamp_trajectory_writer


	def callback(self, data):
		"""
			Reader message callback.
		"""
		#print("data recieved")
		data = list(data.point)

		#delete copies
		data.pop(2)
		data.pop(3)
		
		#print(data)

		self.j += 1

		if self.j == 1:

			#roi_boundaries = [point(data[i].x, data[i].y) 
			#				for i in range(len(data) - 1)]
			roi_boundaries = [point(data[i].x, data[i].y) 
							for i in range(len(data) - 2)]
			
			parking_space = [point_ for point_ in data[1:5]]

			#change roi boundary around the goal 
			#DEBUG achived:
			#roi_boundaries[1].x -= 0.3
			#roi_boundaries[2].x -= 0.3
			#roi_boundaries[3].x += 0.3
			#roi_boundaries[4].x += 0.3

			#roi_boundaries[1].x -= 0.4
			#roi_boundaries[2].x -= 0.4
			#roi_boundaries[3].x += 0.25
			#roi_boundaries[4].x += 0.25

			#roi_boundaries[2].y -= 2
			#roi_boundaries[3].y -= 2


			vehicle_pos = point(data[-2].x, data[-2].y)

			parking_pos = [(data[4].x + data[1].x) / 2, 
						(data[1].y + data[2].y) / 2 + 1]

			#max_steps >= 2!!!!! (utils phis)
			#DEBUG: 170 - 180 steps предел текущий
			#max_steps = 30
			max_steps = args.max_steps
			DEBUG_traj = False
			save_traj = True
			number_image = 32
			
			print("############################")
			print(f"start getting trajectory max_steps = {max_steps}")
			print(f"vehicle pose is: {vehicle_pos.x}, {vehicle_pos.y}")
			print(f"data recieved length: {len(data)}")
			print(time.time())

			isDone, images, trajectory, info_ = get_points(roi_boundaries, 
						vehicle_pos, parking_pos, max_steps=max_steps)

			#print("getting done")
			#print()
			#print()

			#save trajectory
			if save_traj:
				SAVE_DIR = "modules/tools/custom_2/saved_trajectory/"
				
				with open(SAVE_DIR + f"machine_config_{number_image}.txt", 'w') as f:
					f.write(str(trajectory[2][-1]) + " " \
						+ str(info_["car_length"]) + " " \
						+ str(info_["car_width"]) + " " \
						+ str(info_["wheel_base"]))

				with open(SAVE_DIR + f"goals_{number_image}.txt", 'w') as f:
					f.write(str(info_["first_goal"][0]) + " " \
						+ str(info_["second_goal"][0]))
					f.write(str(info_["first_goal"][1]) + " " \
						+ str(info_["second_goal"][1]))

				with open(SAVE_DIR + f"result_points_{number_image}.txt", 'w') as f:
					f.write(" ".join([str(state.x) for state in trajectory[0]]))
					f.write(" ".join([str(state.y) for state in trajectory[0]]))

				with open(SAVE_DIR + f"result_roi_boundaries_{number_image}.txt", 'w') as f:
					f.write(" ".join([str(state.x) for state in roi_boundaries]))
					f.write(" ".join([str(state.y) for state in roi_boundaries]))
				
				with open(SAVE_DIR + f"parking_space_{number_image}.txt", 'w') as f:
					f.write(" ".join([str(state.x) for state in parking_space]))
					f.write(" ".join([str(state.y) for state in parking_space]))

				#print("trajectory saved")
				#print()
				
			trajectory_msg = roi_boundary_message()
			trajectory_msg.timestamp = int(time.time() * 10 ** 7)

			for state, a, phi in zip(trajectory[0], trajectory[1], trajectory[2]):
				next_traj_point = trajectory_msg.point.add()
				next_traj_point.x = state.x
				next_traj_point.y = state.y
				next_traj_point.phi = state.theta
				next_traj_point.v = state.v
				next_traj_point.a = a
				next_traj_point.steer = state.steer
				if DEBUG_traj:
					print(state.x, state.y)
				#добавить steer, acceleration, v, accum_s

			#print()
			#print("average time for action")
			#print(sum(trajectory[3]) / len(trajectory[3]))

			#print("average time for step")
			#print(sum(trajectory[4]) / len(trajectory[4]))

			ten_trag_times_action = []
			ten_trag_times_step = []

			for i in range(10, len(trajectory[3]) + 1, 10):
				ten_trag_times_action.append( sum(trajectory[3][i - 10 : i]) )
				ten_trag_times_step.append( sum(trajectory[4][i - 10 : i]) )

			#print("average time for 10 actions")
			#print(sum(ten_trag_times_action) / len(ten_trag_times_action))

			#print("average time for 10 steps")
			#print(sum(ten_trag_times_step) / len(ten_trag_times_step))

			#print("sending trajectory to Apollo")
			#print()

			self.polamp_trajectory_writer.trajectory_writer.write(trajectory_msg)

			if isDone:
				print("goal was achieved")
			else:
				print("goal was not achieved")
			print("sending done")		
			print("############################")
			

		


if __name__ == "__main__":
	apollo_test = ApolloFeatures()

	seq=0
	time.sleep(2.0)

	map_type = "test_scene" 

	
	routing_1_x = 388930.74	
	routing_1_y = 221211.47

	routing_2_x = 389024.08
	routing_2_y = 221211.47

	starting_distance = 40 # starting distance between the 2 vehicles 

	theta = math.atan2(routing_2_y - routing_1_y,
		    routing_2_x - routing_1_x)
		    
	initial_x = routing_1_x #+ math.cos(theta) 
	#* starting_distance # initial position of obstacle
	initial_y = routing_1_y #+ math.sin(theta) 
	#* starting_distance # initial position of obstacle
	old_x= initial_x 
	old_y= initial_y
	# initial position of obstacle
	initial_x3 = routing_1_x + math.cos(theta) * (starting_distance - 60 )
	# initial position of obstacle
	initial_y3 = routing_1_y + math.sin(theta) * (starting_distance - 60 )
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



	'''
		Добавление: добавление writer node
	'''
	#------------------------------------------------------------------------
	polamp_trajectory_writer = RoiWriter()

	'''
		Добавление: добавление reader node
	'''
	#------------------------------------------------------------------------
	roi_boundary_reader = RoiReader(polamp_trajectory_writer)


	#------------------------------------------------------------------------



	while not cyber.is_shutdown():

		tracking_time = cyber_time.Time.now().to_sec() - start_time
		if (tracking_time > 2 and not parking_req_sent):
			parking_req_sent = True
			parking_msg = RoutingRequest()
			parking_msg.header.module_name = 'dreamview'
			parking_msg.header.sequence_num = 0


	#------------------------------Отправка запроса на планирование-----------------------------------
			#загрузим парковочные места из файла parking_points.txt
			'''
			number_of_place = int(sys.argv[1]) #номер парковочного места, куда происходит парковка
			positions_ = []
			with open("parking_points.txt", 'r') as file:
				print(f"file is opened: {not file.closed}")
				for ind, line in enumerate(file):
					get_coordinate = float(line)
					print(get_coordinate) 
					positions_.append(get_coordinate)
					if ind == 6 * number_of_place - 1: break	

			x_start, y_start, x_end, y_end = positions_[6 * number_of_place - 6], 
			positions_[6 * number_of_place - 5], positions_[6 * number_of_place - 4], 
			positions_[6 * number_of_place - 3]
			center_parkin_place_x, center_parkin_place_y = positions_[6 * number_of_place - 2], 
			positions_[6 * number_of_place - 1]

	#!!!!!!!!!!!!!!!!!!!!!!!ИСПРАВЛЕНИЕ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			x_start = 388932.98
			x_end += 20
			center_parkin_place_x = 388962.98
			'''
		
	
			"""
			x_start = 165888
			y_start = 77.2735

			x_end = 165933
			y_end = 77.2735

			center_parkin_place_x = 165913
			center_parkin_place_y = 72.5563
			number_of_place = 117
			"""
			
			x_start = 388983
			y_start = 221155
			
			x_end = 389018
			y_end = 221155

			center_parkin_place_x = 388999
			center_parkin_place_y = 221151
			number_of_place = 82


			roi_boundary_reader.parking_pos = point(center_parkin_place_x,
													center_parkin_place_y)

			waypoint1 = parking_msg.waypoint.add()
			waypoint1.pose.x = float(x_start)
			waypoint1.pose.y = float(y_start)
			waypoint1 = parking_msg.waypoint.add()
			waypoint1.pose.x = float(x_end)
			waypoint1.pose.y = float(y_end)

			parking_msg.parking_info.parking_point.x = float(center_parkin_place_x)
			parking_msg.parking_info.parking_point.y = float(center_parkin_place_y)

			'''
			#надо изменять парковочное место
			parking_msg.parking_info.parking_space_id = f"place_{number_of_place}"
			apollo_test.routing_writer.write(parking_msg)
			'''

			parking_msg.parking_info.parking_space_id = f"place_{number_of_place}"
			#parking_msg.parking_info.parking_space_id = number_of_place
			apollo_test.routing_writer.write(parking_msg)
			print('PARKING REQUEST')




			

			
