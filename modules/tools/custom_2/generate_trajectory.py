
from math import pi

#class TaskCreater:
#  def __init__():
    

        

def generate_first_goal(d_first_goal, roi_boundaries, 
                        vehicle_pos, parking_pos):
    """
    generate goal for forward task
    """
    d_first_goal_x, d_first_goal_y = d_first_goal
    x = roi_boundaries[4].x + d_first_goal_x
    #x = (roi_boundaries[4].x + roi_boundaries[5].x) / 2
    y = (roi_boundaries[6].y + roi_boundaries[5].y) / 2 \
      + (roi_boundaries[6].y - roi_boundaries[5].y) / 4 + d_first_goal_y
    #y = (roi_boundaries[6].y + roi_boundaries[5].y) / 2 \
    #  + (roi_boundaries[6].y - roi_boundaries[5].y) / 4 - 2.3
    theta = 30 * (pi / 180)
    first_goal = [x, y, theta, 0., 0]
            
    return first_goal


def generate_map(roi_boundaries, vehicle_pos, parking_pos):
    '''
        return list of [left_bottom, right_bottom, top]
    '''
    '''
        roi_boundaries[0] = (roi_boundaries[0].x, roi_boundaries[0].y)
        vehicle_pos = (vehicle_pos.x, vehicle_pos.y)

        create horzontal task:

                            top
        -----------------------------------------------------

                                            first_goal
          start                             
        -------------------      -----------------------------   
         left_bottom        |   |     right_bottom
                            |   |
                            -----
                          down_bottom
    '''                     

    left_bottom = [
                    (roi_boundaries[1].x + roi_boundaries[0].x) / 2, 
                    (roi_boundaries[1].y + roi_boundaries[2].y) / 2,
                    0,
                    (roi_boundaries[1].y - roi_boundaries[2].y) / 2,
                    (roi_boundaries[1].x - roi_boundaries[0].x) / 2
                  ]

    down_bottom = [
                    (roi_boundaries[3].x + roi_boundaries[2].x) / 2, 
                    roi_boundaries[2].y - 2,
                    0,
                    2,
                    (roi_boundaries[3].x - roi_boundaries[2].x) / 2 \
                        + roi_boundaries[5].x - roi_boundaries[4].x
                  ]

    right_bottom = [
                    (roi_boundaries[5].x + roi_boundaries[4].x) / 2, 
                    (roi_boundaries[3].y + roi_boundaries[4].y) / 2,
                    0,
                    (roi_boundaries[4].y - roi_boundaries[3].y) / 2,
                    (roi_boundaries[5].x - roi_boundaries[4].x) / 2
                   ]
    '''
    top = [(roi_boundaries[7].x + roi_boundaries[6].x) / 2, 
            roi_boundaries[7].y + 2,
            0,
            2,
            (roi_boundaries[6].x - roi_boundaries[7].x) / 2]
    '''
    #DEBUG
    top = [(roi_boundaries[7].x + roi_boundaries[6].x) / 2, 
            roi_boundaries[7].y,
            0,
            2,
            (roi_boundaries[6].x - roi_boundaries[7].x) / 2]

    start = [vehicle_pos.x, vehicle_pos.y, 
            0, 0., 0]

    print("DEBUG contrains:")
    print("left_bottom: ", "x_c:", left_bottom[0], 
          "y_c", left_bottom[1], "theta:", left_bottom[2],
          "width:", left_bottom[3], "height:", left_bottom[4])
    print("down_bottom: ", "x_c:", down_bottom[0], 
          "y_c", down_bottom[1], "theta:", down_bottom[2],
          "width:", down_bottom[3], "height:", down_bottom[4])
    print("right_bottom: ", "x_c:", right_bottom[0], 
          "y_c", right_bottom[1], "theta:", right_bottom[2],
          "width:", right_bottom[3], "height:", right_bottom[4])
    print("top: ", "x_c:", top[0], 
          "y_c", top[1], "theta:", top[2],
          "width:", top[3], "height:", top[4])

    return [left_bottom, down_bottom, right_bottom, top]


def create_task(d_first_goal, roi_boundaries, vehicle_pos, 
                parking_pos, dyn_obsts = []):
    map = generate_map(roi_boundaries, vehicle_pos, parking_pos)

    start = [vehicle_pos.x, vehicle_pos.y, 0, 0., 0]
    first_goal = generate_first_goal(d_first_goal, roi_boundaries, 
                            vehicle_pos, parking_pos)
    print("first goal:", first_goal[0], first_goal[1])

    #set task
    is_obs_dyn = False
    if len(dyn_obsts) > 0:
      is_obs_dyn = True
    maps = {}
    trainTask = {}
    valTasks = {}
    maps["map0"] = map
    trainTask["map0"] = [[start, first_goal]]
    valTasks["map0"] = [[start, first_goal]]
    if is_obs_dyn:
      for dyn_obs in dyn_obsts:
        trainTask["map0"][0].append([dyn_obs])
        valTasks["map0"][0].append([dyn_obs])
    
    print("DEBUG generate_traj:", trainTask["map0"][0])
    
    return maps, trainTask, valTasks, \
        [parking_pos[0], parking_pos[1] - 2.2, 90 * (pi / 180), 0, 0]

