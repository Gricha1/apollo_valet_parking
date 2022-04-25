from platform import machine
import numpy as np
import time
import matplotlib.pyplot as plt

def point_shift(point_x, point_y, angle, dist):
  dx_shift = dist * np.cos(angle)
  dy_shift = dist * np.sin(angle)
  return [point_x + dx_shift, point_y + dy_shift]


def get_car_coordinates(list_points, x_points_traj, y_points_traj):
  """
  input:
  list_points - car config = phi(last point), length, width, l_base
  x_points_traj, x_points_traj - current shifted position(center of back axis)
  
  return:
  car_coordinates - list(list)
  len(car_coordinates) = 4
  """

  list_points = list_points.split(" ")
  list_points = [float(x) for x in list_points]

  phi, length, width, l_base = list_points
  car_pose_shifted = [x_points_traj, y_points_traj]
  shift_length = l_base / 2
  dx_shift = shift_length * np.cos(phi)
  dy_shift = shift_length * np.sin(phi)
  car_center = [car_pose_shifted[0] + dx_shift, car_pose_shifted[1] + dy_shift]


  #clockwise
  top_boundary = point_shift(car_center[0], car_center[1], 
                                phi, length / 2)
  bottom_boundary = point_shift(car_center[0], car_center[1], 
                                phi + np.pi, length / 2)
  car_coordinates = []
  car_coordinates.append(point_shift(top_boundary[0], top_boundary[1], 
                                    phi + np.pi / 2, width / 2))
  car_coordinates.append(point_shift(top_boundary[0], top_boundary[1], 
                                    phi - np.pi / 2, width / 2))
  car_coordinates.append(point_shift(bottom_boundary[0], bottom_boundary[1], 
                                    phi - np.pi / 2, width / 2))
  car_coordinates.append(point_shift(bottom_boundary[0], bottom_boundary[1], 
                                    phi + np.pi / 2, width / 2))
  #copy point for plotting
  car_coordinates.append(point_shift(top_boundary[0], top_boundary[1], 
                                    phi + np.pi / 2, width / 2))

  return car_coordinates




def get_coordinates(list_points):
  """
  input: 
  list_points(str)
  car_bounds - if true then calculate 4 points of car only

  return:
  x(list)
  y(list)
  """
  list_points = list_points.split(" ")
  adding_point = list_points.pop(len(list_points) // 2)
  list_points = [float(x) for x in list_points]
  try:
    x = list_points[: len(list_points) // 2] \
    + [float(adding_point[:18])]
  except:
    print("DEBUG")
    print(adding_point[:4])
    #x = list_points[: len(list_points) // 2] \
    #+ [float(adding_point[:17])]
    x = list_points[: len(list_points) // 2] \
    + [float(adding_point[:4])]
    
  #y = [float(adding_point[18:])] \
  #+ list_points[len(list_points) // 2 :]
  y = [float(adding_point[:4])] \
  + list_points[len(list_points) // 2 :]

  return x, y


number_image = 32
LOAD_DIR = "/home/reedgern/apollo_third/apollo6.0/modules/tools/custom_2/"

machine_config = f"saved_trajectory/machine_config_{number_image}.txt"
with open(LOAD_DIR + machine_config, 'r') as f:
  list_machine_config = f.read()

goals = f"saved_trajectory/goals_{number_image}.txt"
with open(LOAD_DIR + goals, 'r') as f:
  list_goals = f.read()

traj = f"saved_trajectory/result_points_{number_image}.txt"
with open(LOAD_DIR + traj, 'r') as f:
  list_traj = f.read()

obst = f"saved_trajectory/result_roi_boundaries_{number_image}.txt"
with open(LOAD_DIR + obst, 'r') as f:
  list_obst = f.read()

parking_space = f"saved_trajectory/parking_space_{number_image}.txt"
with open(LOAD_DIR + parking_space, 'r') as f:
  list_park = f.read()

x_goals, y_goals = get_coordinates(list_goals)
x_points_park, y_points_park = get_coordinates(list_park)
x_points_obst, y_points_obst = get_coordinates(list_obst)
x_points_traj, y_points_traj = get_coordinates(list_traj)
x_points_traj.pop(0)
y_points_traj.pop(0)

car_current_coordinates = \
  get_car_coordinates(list_machine_config, 
                      x_points_traj[-1], 
                      y_points_traj[-1])
car_current_x = [p[0] for p in car_current_coordinates]
car_current_y = [p[1] for p in car_current_coordinates]

#car_current_coordinates = \
#  get_car_coordinates(list_machine_config, 
#                      x_points_traj[-1], 
#                      y_points_traj[-1])
#car_current_end = [p[0] for p in car_current_coordinates]
#car_current_end = [p[1] for p in car_current_coordinates]

plt.plot(car_current_x, car_current_y)
plt.plot(x_points_traj, y_points_traj)

x_points_obst.pop(0)
y_points_obst.pop(0)
plt.plot(x_points_obst, y_points_obst)
#plt.plot(x_points_park, y_points_park)
#plt.scatter(x_goals[0], y_goals[0], c='red')
plt.scatter(x_goals[1], y_goals[1], c='red')
plt.scatter(x_points_traj[-1], y_points_traj[-1], c='green')

plt.grid()
plt.show()

None

