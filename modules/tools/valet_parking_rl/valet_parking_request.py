#!/usr/bin/env python3
from ast import arg
import sys
import time
import os
import json
from cyber.python.cyber_py3 import cyber, cyber_time
import argparse
import http.client
sys.path.insert(0, "modules/tools/valet_parking_rl/POLAMP_sample_factory_/")
from rl_utils.rl_utils import create_task
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.planning.proto.planning_pb2 import roi_boundary_message

parser = argparse.ArgumentParser(description='Get max steps for validation')
parser.add_argument('-max_steps', '--max_steps', type=int, 
                    help='max_steps = steps in env')
parser.add_argument('-map', '--map')
parser.add_argument('-parking_place', '--parking_place', type=int)
parser.add_argument('-get_a_star_trajectory', '--get_a_star_trajectory', default=False,
					type=bool)
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

class RlTrajectoryWriter:
    def __init__(self):
        cyber.init()
        self.node = cyber.Node("roi_writer")
        self.trajectory_writer = self.node.create_writer('from_python_to_apollo', 
														 roi_boundary_message)

class RoiBoundariesReaderAndTrajecoryWriter:
	def __init__(self, rl_trajectory_writer):
		cyber.init()
		self.node = cyber.Node("listener")
		self.reader = self.node.create_reader('get_roi_boundaries_topic', 
										roi_boundary_message, self.callback)
		self.count_recieved_messages = 0
		self.rl_trajectory_writer = rl_trajectory_writer
		self.missed_count = 0
		self.get_a_star_trajectory= False

	def callback(self, data):
		if self.missed_count < 10:
			print("debug test python, miss count:", self.missed_count)
			self.missed_count = self.missed_count + 1
			return

		if self.get_a_star_trajectory:
			print("DEBUG A* trajectory")
			trajectory_msg = roi_boundary_message()
			trajectory_msg.timestamp = int(time.time() * 10 ** 7)
			self.rl_trajectory_writer.trajectory_writer.write(trajectory_msg)
			return

		self.count_recieved_messages += 1
		if self.count_recieved_messages == 1:
			start_time_get_trajectory = time.time()
			task = create_task(data)
			print("debug generated task")
			print("task:", task)
			request_ = json.dumps({"task" : task})

			c = http.client.HTTPConnection('172.17.0.2', 8080)
			c.request('POST', '/process', request_)

			respond_ = c.getresponse().read()
			respond_ = json.loads(respond_)

			rl_trajectory_info = respond_["trajectory"]
			rl_run_info = respond_["run_info"]
			print("generated trajectory status:", rl_run_info)
			print("trajectory len:", len(rl_trajectory_info["x"]))
			print("first 5 points:")
			print("x:", rl_trajectory_info["x"][0:5])
			print("y:", rl_trajectory_info["y"][0:5])
			print("v:", rl_trajectory_info["v"][0:5])
			print("steer:", rl_trajectory_info["steer"][0:5])
			print("heading:", rl_trajectory_info["heading"][0:5])
			print("a:", rl_trajectory_info["accelerations"][0:5])
			print("w:", rl_trajectory_info["w"][0:5])
			print("v_s:", rl_trajectory_info["v_s"][0:5])
			print("last 5 points:")
			print("x:", rl_trajectory_info["x"][-5:])
			print("y:", rl_trajectory_info["y"][-5:])
			print("v:", rl_trajectory_info["v"][-5:])
			print("steer:", rl_trajectory_info["steer"][-5:])
			print("heading:", rl_trajectory_info["heading"][-5:])
			print("a:", rl_trajectory_info["accelerations"][-5:])
			print("w:", rl_trajectory_info["w"][-5:])
			print("v_s:", rl_trajectory_info["v_s"][-5:])
			
			trajectory_msg = roi_boundary_message()
			trajectory_msg.timestamp = int(time.time() * 10 ** 7)
			for x, y, v, steer, phi, a, w, v_s in zip(
									  rl_trajectory_info["x"],
			                          rl_trajectory_info["y"],
									  rl_trajectory_info["v"],
									  rl_trajectory_info["steer"],
									  rl_trajectory_info["heading"], 
									  rl_trajectory_info["accelerations"],
									  rl_trajectory_info["w"],
									  rl_trajectory_info["v_s"]):
				next_traj_point = trajectory_msg.point.add()
				next_traj_point.x = x
				next_traj_point.y = y
				next_traj_point.v = v
				next_traj_point.steer = steer
				next_traj_point.phi = phi
				next_traj_point.a = a
				next_traj_point.w = w
				next_traj_point.v_s = v_s

			end_time_get_trajectory = time.time()
			print("time for geting trajectory in Python:", 
					end_time_get_trajectory - start_time_get_trajectory)
			self.rl_trajectory_writer.trajectory_writer.write(trajectory_msg)
			
		return
			
if __name__ == "__main__":
	apollo_valet_parking_interface = ApolloValetParkingRequestInterface()
	second_req_sent = False
	parking_req_sent = False
	start_time = cyber_time.Time.now().to_sec()
	rl_trajectory_writer = RlTrajectoryWriter()
	roi_boundary_reader = RoiBoundariesReaderAndTrajecoryWriter(rl_trajectory_writer)
	if args.get_a_star_trajectory:
		roi_boundary_reader.get_a_star_trajectory = True
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
			parking_msg.parking_info.parking_space_id = f"place_{number_of_place}"
			apollo_valet_parking_interface.routing_writer.write(parking_msg)
			print('PARKING REQUEST')
			print('parking map:', args.map)
			print('parking place:', number_of_place)




				

				
