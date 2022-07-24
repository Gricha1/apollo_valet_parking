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
parser.add_argument('-dence_prob', '--dence_prob')
parser.add_argument('-dynamic', '--dynamic')
parser.add_argument('-map', '--map')
parser.add_argument('-parking_number', '--number_of_place', type=int)
#parser.add_argument('-test_case', '--test_case', type=int)
args = parser.parse_args()
p = float(args.dence_prob)
dynamic = True
if args.dynamic == "0":
    dynamic = False

ego = lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis
#map = "95088a4f-4dbc-49b8-b7c8-b781d060de52"
#map = "736709ef-8a65-46e2-ae32-d564cf5753b8"
#map = "08a90728-c32c-40b2-ac51-d065b31f5aab"
if args.map == "train":
    map = "95088a4f-4dbc-49b8-b7c8-b781d060de52"
else:
    map = "736709ef-8a65-46e2-ae32-d564cf5753b8"
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

#count_of_parkining_places = 17
#count_of_fill_places = int(np.floor((count_of_parkining_places * p)))
#nums_of_fill_places = np.random.choice(np.arange(count_of_parkining_places),
#                                     size=count_of_fill_places, replace=False)
#indeces_of_fill_places = sorted(nums_of_fill_places)


#Добавляем нпс
'''
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
'''
#--------------------------------------------------Конфигурация препрядствий-----

class to_pos:
    def __init__(self, dx, dz, orientation, row, col):
        self.dx = dx
        self.dz = dz
        self.orientation = orientation
        self.row = row
        self.col = col

#to_place[i][j] - приращение которое нужно добавить к начальной позиции spawn[0] 
# чтобы попасть в i, j парковочное место
#x = -4.9 #первый ряд
#x = 8 #второй ряд
#x = 12.5 #третий ряд
#x = 25.4 #четвертый ряд
#orientation = -1 #-1 ориентация препядствия вниз
#x -= 4.9 #первое парковочное место
#z = 0.5 - 2.7 * 23 #первое парковочное место
'''
row_count_places = 4
column_count_places = 24
to_place = []
for i in range(row_count_places):
    for j in range(column_count_places):
        if i == 0:
            orientation = -1
            dx = -6.4
        elif i == 1:
            orientation = 1
            dx = 8
        elif i == 2:
            orientation = -1
            dx = 12.5
        else:
            orientation = 1
            dx = 25.4
        dz = 0.5 - 2.7 * 23 + 2.7 * j
        to_place.append(to_pos(dx, dz, orientation, i, j))
        

count_of_parkining_places = row_count_places * column_count_places
count_of_fill_places = int(np.floor((count_of_parkining_places * p)))
random_to_place = np.random.choice(to_place, size=count_of_fill_places, replace=False)


number_of_place = args.number_of_place
#number_of_place = 82
ind_row_of_place = number_of_place // column_count_places
ind_col_of_place = number_of_place % column_count_places - 1

#------------------------------------------------------Добавление НПС---------------------
for pos in random_to_place:
    npcState = lgsvl.AgentState()
    spawns = sim.get_spawn()
    npcState.transform = spawns[0]
    npcState.transform.position.x += pos.dx
    npcState.transform.position.z += pos.dz
    npcState.transform.rotation.y += pos.orientation * 90
    if pos.row == row_count_places - 1 and pos.col == 0:
        continue
    if pos.row == row_count_places - 1 and pos.col == 1:
        continue
    if not(pos.row == ind_row_of_place and pos.col == ind_col_of_place):
        sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState)


'''
#----------------------------------------------------------




#------------------------Добавление нпс_1 машины---------------------
#npcState = lgsvl.AgentState()
#npcState.transform = egoState.transform
#npcState.transform.position = egoState.position + 5.8 * right
#npcState.transform.position = egoState.position + 22.5 * forward
#npcState.transform.position.y += 0.5  
#npcState.transform.rotation.y -= 90
#npc = sim.add_agent("Jeep", lgsvl.AgentType.NPC, npcState)
#npcState.transform.position = egoState.position - 5.8 * right
#npcState.transform.position = egoState.position - 22.5 * forward
#npcState.transform.position.y -= 0.5  
#npcState.transform.rotation.y += 90



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
#Начальное движение для НПС 3
#if case == "1":
#    s = npcState_3
#    s.velocity.y = 10
#    npc_3.state = s
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

t0 = time.time()

npcChangedLanes = False

test_case = np.random.randint(27)

right_shift = np.linspace(19.5, 18, 3)
forward_shift = np.linspace(34, 38, 3)
speed_obs = np.linspace(0.8, 1.2, 3)
right_shift_ = right_shift[test_case % 3]
if test_case // 3 >= 2:
    forward_shift_ = forward_shift[2]
else:
    forward_shift_ = forward_shift[test_case // 3]

if test_case // 9 >= 2:
    speed_obs_ = speed_obs[2]
else:
    speed_obs_ = speed_obs[test_case // 9]

follow_lane = False
roi_follow_lane = False
polamp_follow_lane = False
right_shift_ = 2.3
forward_shift_ = 30
#right_shift_ = 19.7 #working
#forward_shift_ = 35 #working
speed_obs_ = 1 # wroking
#right_shift_ = 19.5
#forward_shift_ = 34
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

    
