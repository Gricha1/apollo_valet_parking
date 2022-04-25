#!/usr/bin/env python3

import os
import lgsvl
import time
import sys
import numpy as np
import argparse
from cyber.python.cyber_py3 import cyber, cyber_time
from modules.planning.proto.planning_pb2 import roi_boundary_message


class POLAMP_ready_reader:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("PythonAPI_POLAMP_listener")
        self.reader = self.node.create_reader('from_python_to_apollo', 
                                    roi_boundary_message, self.callback)
        self.ready_create_car = False
        self.car_created = False

    def callback(self, data):
        #print("debug_python_api:", "trajectory ready")
        self.ready_create_car = True

class ROI_boundary_ready_reader:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("PythonAPI_RoiBoundary_listener")
        self.reader = self.node.create_reader('get_roi_boundaries_topic', 
                                    roi_boundary_message, self.callback)
        self.ready_create_car = False
        self.car_created = False

    def callback(self, data):
        #print("debug_python_api:", "roi boundary ready")
        #data = list(data.point)
        #print("len roi boundary:", len(data))
        self.ready_create_car = True
       
polamp_traj_check_reader = POLAMP_ready_reader()
roi_boundary_check_reader = ROI_boundary_ready_reader()

parser = argparse.ArgumentParser()
parser.add_argument('-p', '--prob')
parser.add_argument('-d', '--dynamic')
args = parser.parse_args()
p = float(args.prob)
dynamic = True
if args.dynamic == "0":
    dynamic = False

ego = lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis
#map = "95088a4f-4dbc-49b8-b7c8-b781d060de52"
#map = "736709ef-8a65-46e2-ae32-d564cf5753b8"
#map = "08a90728-c32c-40b2-ac51-d065b31f5aab"
map = "95088a4f-4dbc-49b8-b7c8-b781d060de52"
SIMULATOR_HOST = os.environ.get("LGSVL__SIMULATOR_HOST", "127.0.0.1")
SIMULATOR_PORT = int(os.environ.get("LGSVL__SIMULATOR_PORT", 8181))
BRIDGE_HOST = os.environ.get("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
BRIDGE_PORT = int(os.environ.get("LGSVL__AUTOPILOT_0_PORT", 9090))
sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
if sim.current_scene == map:
    sim.reset()
else:    
    sim.load(map, seed=0)

#------------------------------Добавление Агента------------------------------
vehicle_id = "70f2a06e-2b44-45bb-a6d7-d1d1b8813bcc"
spawns = sim.get_spawn()
egoState = lgsvl.AgentState()
egoState.transform = spawns[0]
#print (spawns[0].rotation.y)
ego = sim.add_agent(vehicle_id, lgsvl.AgentType.EGO, egoState)
ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
right = lgsvl.utils.transform_to_right(egoState.transform)
forward = lgsvl.utils.transform_to_forward(egoState.transform)

#----------------------------------------------------------

count_of_parkining_places = 17
count_of_fill_places = int(np.floor((count_of_parkining_places * p)))
nums_of_fill_places = np.random.choice(np.arange(count_of_parkining_places),
                                     size=count_of_fill_places, replace=False)
indeces_of_fill_places = sorted(nums_of_fill_places)

'''
#Добавляем нпс
npcState = lgsvl.AgentState()
npcState.transform = egoState.transform
npcState.transform.position.y += 0.5
npcState.transform.rotation.y += -90
npcState.transform.position = egoState.position + 22 * forward + 7 * right
for i in range(len(indeces_of_fill_places)):
    npcState_i = lgsvl.AgentState()
    npcState_i.transform = egoState.transform
    ind_current = indeces_of_fill_places[i]
    if i != 0:
        ind_last = indeces_of_fill_places[i - 1]
        npcState_i.transform.position = egoState.position - 4 * ind_last * forward
    npcState_i.transform.position = egoState.position + 4 * ind_current * forward
    sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_i)
    



#----------------------------------------------------------
'''



#------------------------Добавление нпс_1 машины---------------------
#npcState.transform = egoState.transform
#npcState.transform.position = egoState.position + 3.6 * right
#npcState.transform.position = egoState.position + 7 * forward
#npcState.transform.position.y += 0.5  
#npcState.transform.rotation.y -= 90
#npc = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState)



#------------------------Добавление нпс 3 машины--------------------

'''
npcState_3 = lgsvl.AgentState()
npcState_3.transform = egoState.transform
npcState_3.transform.position = egoState.position - 3 * right
npcState_3.transform.position = egoState.position + 40 * forward
npcState_3.transform.rotation.y -= 180
npc_3 = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_3)
npcState_3.transform.position = egoState.position + 3 * right
npcState_3.transform.position = egoState.position - 40 * forward
npcState_3.transform.rotation.y -= -180
Начальное движение для НПС 3
if case == "1":
    s = npcState_3
    s.velocity.y = 10
    npc_3.state = s
'''

#----------------Добавление нпс 2 машины----------------------------------------
"""
npcState_2 = lgsvl.AgentState()
npcState_2.transform = egoState.transform
npcState_2.transform.position = egoState.position - 5.5 * forward
npc_2 = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_2)


#Начальное движение для НПС 2
if case == '2':
    s = npcState_2
    s.velocity.x = -1   
    npc_2.state = s


"""

# This function will be called if a collision occurs
def on_collision(agent1, agent2, contact):
    raise Exception("{} collided with {}".format(agent1, agent2))

controlReceived = False
# This function will be called when the Simulator receives 
# the first message on the topic defined in the CheckControlSensor configuration
def on_control_received(agent, kind, context):
    global controlReceived
    # There can be multiple custom callbacks defined, 
    # this checks for the appropriate kind
    if kind == "checkControl":
        # Stops the Simulator running, this will only interrupt 
        # the first sim.run(30) call
        sim.stop()
        controlReceived = True

ego.on_custom(on_control_received)

# Run Simulator for at most 30 seconds for the AD stack to to initialize
#sim.run(1000)

# If a Control message was not received, then the AD stack 
# is not ready and the scenario should not continue
#if not controlReceived:
#    raise Exception("AD stack is not ready after 30 seconds")
#    sys.exit()

# NPC will follow the HD map at a max speed of 15 m/s (33 mph) 
# and will not change lanes automatically
# The speed limit of the road is 20m/s so the EGO should drive 
# faster than the NPC


#-----------------------Движение npc--------------------------------------
#npc_2.follow_closest_lane(follow=True, max_speed=8, isLaneChange=False)
#npc_2.follow(waypoints)



# t0 is the time when the Simulation started
t0 = time.time()

# This will keep track of if the NPC has already changed lanes
npcChangedLanes = False

# Run Simulation for 4 seconds before checking cut-in or end conditions
#sim.run(4)

# The Simulation will pause every 0.5 seconds to check 2 conditions
follow_lane = False
roi_follow_lane = False
polamp_follow_lane = False
right_shift_ = 1.3
forward_shift_ = 30
speed_obs_ = 1
roi_speed_sended = 0

while True:
    sim.run(0.1)
    if time.time() - t0 > 100000:
            break
    
    if roi_boundary_check_reader.ready_create_car \
        and not roi_boundary_check_reader.car_created and dynamic:
        #first npc
        print("debug bride: generate first nps")
        npcState_temp = lgsvl.AgentState()
        npcState_temp.transform = egoState.transform
        npcState_temp.transform.position = egoState.position - \
                                                    right_shift_ * right
        npcState_temp.transform.position = egoState.position \
                                                    + forward_shift_ * forward
        npcState_temp.transform.rotation.y -= 180
        npc_before_polamp = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_temp)
        #npc.follow_closest_lane(
        #                                follow=True, 
        #                                max_speed=2, 
        #                                isLaneChange=False
        #                            )
        #return to origin point


        npcState_temp.transform = egoState.transform
        npcState_temp.transform.position = egoState.position \
                                            + right_shift_ * right
        npcState_temp.transform.position = egoState.position \
                                                    - forward_shift_ * forward
        npcState_temp.transform.rotation.y -= -180
        roi_boundary_check_reader.car_created = True

    if polamp_traj_check_reader.ready_create_car \
        and not polamp_traj_check_reader.car_created and dynamic:
        print("debug bride: generate second nps")
        npcState_temp = lgsvl.AgentState()
        npcState_temp.transform = egoState.transform
        npcState_temp.transform.position = egoState.position \
                                                - right_shift_ * right
        npcState_temp.transform.position = egoState.position \
                                                + forward_shift_ * forward
        npcState_temp.transform.rotation.y -= 180
        npc_after_polamp = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_temp)
        #npc_after_polamp.follow_closest_lane(
        #                                follow=True, 
        #                                max_speed=2, 
        #                                isLaneChange=False
        #                            )
        #return to origin point
        npcState_temp.transform = egoState.transform
        npcState_temp.transform.position = egoState.position \
                                                    + right_shift_ * right
        npcState_temp.transform.position = egoState.position \
                                                    - forward_shift_ * forward
        npcState_temp.transform.rotation.y -= -180
        polamp_traj_check_reader.car_created = True

    if roi_boundary_check_reader.ready_create_car \
        and roi_boundary_check_reader.car_created and dynamic:
        roi_speed_sended += 1
        if polamp_traj_check_reader.ready_create_car \
            or roi_speed_sended >= 15:
            s = npc_before_polamp.state
            s.velocity.z = -10
            s.velocity.x = -10
            #s.velocity.y = 10
            npc_before_polamp.state = s
            #npc_before_polamp.follow_closest_lane(
            #                            follow=True, 
            #                            max_speed=2, 
            #                            isLaneChange=False
            #                        )
        else:
            s = npc_before_polamp.state
            s.velocity.z = -speed_obs_
            npc_before_polamp.state = s

    if polamp_traj_check_reader.ready_create_car \
        and polamp_traj_check_reader.car_created and dynamic:
        s = npc_after_polamp.state
        s.velocity.z = -speed_obs_
        npc_after_polamp.state = s
    

        '''
        #second npc
        npcState_temp_1 = lgsvl.AgentState()
        npcState_temp_1.transform = egoState.transform
        npcState_temp_1.transform.position = egoState.position + 3 * right
        npcState_temp_1.transform.position = egoState.position + 2 * forward
        npcState_temp_1.transform.rotation.y += 90
        npc_1 = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_temp_1)
        npc_1.follow_closest_lane(
                                        follow=True, 
                                        max_speed=2, 
                                        isLaneChange=False
                                    )
        #return to origin point
        npcState_temp_1.transform = egoState.transform
        npcState_temp_1.transform.position = egoState.position + 3 * right
        npcState_temp_1.transform.position = egoState.position + 2 * forward
        npcState_temp_1.transform.rotation.y -= 90
        roi_boundary_check_reader.car_created = True
        polamp_traj_check_reader.car_created = True
        '''
    
    '''
    if roi_boundary_check_reader.npc and not roi_follow_lane:
        roi_boundary_check_reader.npc.follow_closest_lane(
                                                            follow=True, 
                                                            max_speed=2, 
                                                            isLaneChange=False
                                                        )
        roi_follow_lane = True
    if polamp_traj_check_reader.npc and not polamp_follow_lane:
        polamp_traj_check_reader.npc.follow_closest_lane(
                                                            follow=True, 
                                                            max_speed=2, 
                                                            isLaneChange=False
                                                        )
        polamp_follow_lane = True
    '''
    #if time.time() - t0 > 10 and not polamp_is_ready:
    #    npcState_temp = lgsvl.AgentState()
    #    npcState_temp.transform = egoState.transform
    #    npcState_temp.transform.position = egoState.position - 3 * right
    #    npcState_temp.transform.position = egoState.position + 40 * forward
    #    npcState_temp.transform.rotation.y -= 180
    #    npc_temp = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState_temp)
    #    npc_temp.follow_closest_lane(follow=True, max_speed=2, isLaneChange=False)
    #    polamp_is_ready = True

    #if time.time() - t0 > 5 and not follow_lane:
        #if case == '2': npc_2.follow_closest_lane(follow=True, 
        #                               max_speed=8, isLaneChange=False)
        #if case == '1': npc_3.follow_closest_lane(follow=True, 
        #                                max_speed=2, isLaneChange=False)
        #follow_lane = True

'''
    # If the NPC has not already changed 
    # lanes then the distance between the NPC and EGO is calculated
    
    if not npcChangedLanes:
        egoCurrentState = ego.state
        npcCurrentState = npc.state

        separationDistance = (egoCurrentState.position - \
                            npcCurrentState.position).magnitude()

        # If the EGO and NPC are within 15m, then NPC will change 
        # lanes to the right (in front of the EGO)
        if separationDistance <= 15:
            npc.change_lane(False)
            npcChangedLanes = True
    
    '''

    
