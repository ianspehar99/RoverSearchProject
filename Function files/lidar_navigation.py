# Lidar navigation functions for rover

import numpy as np


# LIDAR NODE FUNCTIONS: Process the raw lidar data, publishes the front distance and side distance
# Could add avgs later
def get_front_distance(scan_array):
    front_i = len(scan_array)//2
    front_distance = scan_array[front_i]

    return front_distance
    

def get_left_distance(scan_array):   # can add min_angle/max_angle specs for lidar if needed
    # angle_range = abs(max_angle)+abs(min_angle)

    # angle_increment = angle_range/(len(scan_array))

    # Starting with scan angle set to -90 to 90, easy for now:
    left_distance = scan_array[-1]

    return left_distance


# LIDAR CALLBACK FUNCTIONS: Inputs are the front and left distances, these handle stopping/bug mode

# Func determines if front lidar value within range, returns og distance that we are going to 
# use as reference when gping around obstacle
def obstacle_ahead(front_lidar_val,stop_threshold = 1):
    
    if front_lidar_val < stop_threshold:
        return True, front_lidar_val
    else: 
        return False

# If the obstacle is ahead, see if we should start bug mode or skip to next waypoint
# If we are already close to waypoint, probably not worth going around obstacle
def start_bug_mode(current_position, waypoint,cutoff_distance=2.5):
    xc = current_position[0]
    yc = current_position[1]
    xw = waypoint[0]
    yw = waypoint[1]
    distance = dist_b2_points(xc, xw, yc, yw)

    hitpoint = current_position
    return distance < cutoff_distance , hitpoint


# Adjusts angle to keep the side lidar distance the same as the original stop distance
# so that rover remains perpendicular to the obstacle wall
def angle_adjuster(side_lidar_val, desired_lidar_dist ,threshold = 0.1,scale_factor = 20):
    # Bool, only turn if diference is above threshold
    turn = False
    turn_angle = 0

    # Get offset from desired position
    offset = side_lidar_val - desired_lidar_dist

    if offset > threshold:

        # Positive angle turns to left!!
        turn_angle = offset*scale_factor

        turn = True
        return turn_angle, turn
    
    else:
        return turn_angle, turn





# This function is for when we are going around obstacle, going to be checking for when we reach the m-line
# and can continue to the waypoint. Returns true or false
def on_m_line(current_position, hit_point,waypoint,distance_threshhold = 1,angle_threshold=0.1):
    # Get the waypoint vector
    dx1 = waypoint[0] - hit_point[0]
    dy2 = waypoint[1] - hit_point[1]

    # Get the current position vector
    dx2 = current_position[0] - hit_point[0]
    dy2 = current_position[1] - hit_point[1]

    # Calc angle between the two vectors
    dot_product = dx1 * dx2 + dy2 * dy2
    mag1 = (dx1 ** 2 + dy2 ** 2) ** 0.5
    mag2 = (dx2 ** 2 + dy2 ** 2) ** 0.5
    angle = dot_product / (mag1 * mag2)
    angle = np.arccos(angle)  # Radias

    dist_from_hitpoint = dist_b2_points(current_position[0],hit_point[0],current_position[1], hit_point[1])

    if angle < angle_threshold and dist_from_hitpoint > distance_threshhold:
        # Checks if our current vectr is close to the waypoint vector, and if we are sufficently far from og hit point (so it wont trigger immediately)
        return True, dist_from_hitpoint, angle
    else:
        return False, dist_from_hitpoint, angle


















# Pythagorean thm btichess
def dist_b2_points(x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2+(y2-y1)**2)
    





