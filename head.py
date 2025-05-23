import math
import tf_transformations
from geometry_msgs.msg import Quaternion


def get_turn_angle(rover_x, rover_y, rover_yaw_deg, goal_x, goal_y):
    # Calculate vector from rover to waypoint
    dx = goal_x - rover_x
    dy = goal_y - rover_y

    nav_vector = (dx, dy) #Will use this later for Bug 2(Check if on the vector)

    # Desired heading 
    desired_heading_rad = math.atan2(dy, dx)
    desired_heading_deg = math.degrees(desired_heading_rad)

    # Calc smallest distance
    theta_dif = desired_heading_deg - rover_yaw_deg
    if theta_dif > 180:
        theta_dif -= 360
    elif theta_dif < -180:
        theta_dif += 360

    return theta_dif, nav_vector # Degrees


# Reached waypoint yet? (Within margin of error radius_)
def reached_waypoint(current_position, waypoint,margin_of_error=0.5):
    xc = current_position[0]
    yc = current_position[1]
    xw = waypoint[0]
    yw = waypoint[1]
    distance = ((xc - xw) ** 2 + (yc - yw) ** 2) ** 0.5
    
    return distance < margin_of_error

def quaternion_to_yaw(quaternion):
    # Convert quaternion to yaw angle
    q = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    euler = tf_transformations.euler_from_quaternion(q)
    bearing = euler[2]  #Bearing in radians
    return bearing, math.degrees(bearing)  

rover_x = 0.0
rover_y = 0.0
rover_yaw_deg = -134
goal_x = 1.0
goal_y = 1.0
angle = get_turn_angle(rover_x, rover_y, rover_yaw_deg, goal_x, goal_y)
print(f"Angle to turn: {angle} degrees")

