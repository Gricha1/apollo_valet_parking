
from math import pi

def generate_first_goal(roi_boundaries, vehicle_pos, parking_pos):
        """
        generate goal for forward task
        """
        
        x = roi_boundaries[4].x + 4
        y = (roi_boundaries[6].y + roi_boundaries[5].y) / 2 \
          + (roi_boundaries[6].y - roi_boundaries[5].y) / 4 - 1.1
        theta = 20 * (pi / 180)
        first_goal = [x, y, theta, 0., 0]
                
        return first_goal


def create_task(roi_boundaries, vehicle_pos, parking_pos, first_goal):
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
    '''
    left_bottom = [(roi_boundaries[1].x + roi_boundaries[0].x) / 2, 
                         roi_boundaries[1].y - 2,
                         0,
                         2,
                         (roi_boundaries[1].x - roi_boundaries[0].x) / 2]

    right_bottom = [(roi_boundaries[5].x + roi_boundaries[4].x) / 2, 
                         roi_boundaries[5].y - 2,
                         0,
                         2,
                         (roi_boundaries[5].x - roi_boundaries[4].x) / 2]

    top = [(roi_boundaries[7].x + roi_boundaries[6].x) / 2, 
            roi_boundaries[7].y + 2,
            0,
            2,
            (roi_boundaries[7].x - roi_boundaries[6].x) / 2]

    start = [vehicle_pos.x, vehicle_pos.y, 
            0, 0., 0]

    #set frist goal
    #first_goal = [(roi_boundaries[5].x + roi_boundaries[4].x) / 2, 
    #    (roi_boundaries[7].y + roi_boundaries[0].y) / 2 + (roi_boundaries[7].y - roi_boundaries[0].y) / 4, 
    #            0, 0., 0]
    #first_goal = [(roi_boundaries[5].x + roi_boundaries[4].x) / 2, 
    #    (roi_boundaries[6].y + roi_boundaries[5].y) / 2 \
    #    + (roi_boundaries[6].y - roi_boundaries[5].y) / 4 - 0.4, 
    #            0, 0., 0]
    #first_goal = [roi_boundaries[4].x + 4, 
    #    (roi_boundaries[6].y + roi_boundaries[5].y) / 2 \
    #    + (roi_boundaries[6].y - roi_boundaries[5].y) / 4 - 0.5, 
    #            0, 0., 0]

    print("first goal:", first_goal[0], first_goal[1])

    #set task
    maps = {}
    trainTask = {}
    valTasks = {}
    maps["map0"] = [left_bottom, right_bottom, top]
    trainTask["map0"] = [[start, first_goal]]
    valTasks["map0"] = [[start, first_goal]]
    
    
    return maps, trainTask, valTasks

