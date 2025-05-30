#!/usr/bin/env python3

# Main Navigation Node for Autonomous Search

# Subscribes to GPS, IMU, and Lidar data
# Navigates robot to waypoints, and goes around obstacles when needed using bug mode
# Publishes linear velocity and deired headings to the cmd node which actually turns the robot

# Ian Spehar

import rclpy
import numpy as np
import csv
from rclpy.node import Node

# Import srv for starting/stopping
from services_messages.srv import SendData

from std_msgs.msg import bool

# Image datatype
from sensor_msgs.msg import Image

# Velocity commands and IMU data
from geometry_msgs.msg import Twist, Quaternion

# (x,y), quaternion for yaw position from Gazebo:
from gazebo_msgs.msg import ModelState
# Access later with msg.pose[index].position.x
#                   msg.pose[index].position.y
# index = msg.name.index("your_robot_name")
# Functions
from head import get_turn_angle, reached_waypoint, quaternion_to_yaw, get_smallest_angle
from lidar_navigation import *   # Need all functions 
import goal_finder

class NavNode(Node):
    def __init__(self):
    
        super().__init__('main_nav_node')

        # SUBSCRIPTIONS (msg type,topic name,callback,queue size)
        # Sub to flag search node
        self.flag_node_sub = self.create_subscription(bool,'flag_found',self.flag_node_callback)

        # GPS and IMU data (For my project, simple x,y - for rover, will be actual gps)
        self.gps_sub = self.create_subscription(ModelState, '/gazebo/model_states',self.gps_callback,10) 
        
        # IMU data - This will be seperate on actual rover, for now the gps_sub gets x,y, and quat from gazebo
        self.imu_sub = self.create_subscription(ModelState, 'heading',self.imu_callback,10)


        # PUBLISHERS
        self.heading_pub = self.create_publisher(Twist, 'heading_and_velocity',10)

        # Creates service  (type,ros2 call name,callback)
        self.service = self.create_service(SendData, 'startstop', self.service_callback)

        # Create timer callback (every 0.1 seconds)
        self.timer = self.create_timer(0.1,self.timer_callback)

        #VARIABLES:
        self.waypoints = [(10,0),(0,10),(-10,0),(-3,-7),(4,0),(0,4),(-4,0)] # List of waypoints to navigate to
        self.i = 0  # Index to track which waypoint we are on
        self.x = 0  # Current x position of the rover
        self.y = 0  # Current y position of the rover
        self.heading = 0 # Current heading of the rover (degrees)
        self.in_bug_mode = False # Flag to indicate if we are in bug mode
        self.waypoint_bound = False # Flag to indicate if goal waypoint is currently set
        self.straight_vel = 2 # Linear velocity
        self.bug_velocity = 1 # Bug velocity
        self.bug_stop_and_turn = False # For when we first enter bug mode
        self.ref_distance = 0 # OG front ref distance that want to keep side distance at
        self.current_side_distance = 0 # Current distance from the side
        self.side_dist_array = [] # For corner handling
        self.hitpoint = ()  # Stores robot position at time of obstacle detected/ bug started
        self.desired_angle = None # store for turning/ angle comparison
        self.turning_left = False
        self.left_clear_heading = None
        self.flag_found = False # Tracks flag finding node output

        self.get_logger().info('Nav node ready, waiting for service call...')

    # Timer callback: The main code that will loop continously
    def timer_callback(self):

        # 1. First, check if we are near the current goal waypoint (Default thresh at 0.5)
        # Function takes in (tuple of rover position) , (tuple of current waypoint coords)
        if reached_waypoint((self.x,self.y),self.waypoints[self.i]):
            self.i += 1 # Update waypoint index for next goal

            # Need new heading, change this so that 'if statement 2;'is triggered
            self.waypoint_bound = False 

            # Stop bug mode if we are near waypoint
            self.in_bug_mode = False

        # 2. Set next goal waypoint and heading if needed 
        # (triggered at beginning and after we reach waypoints)
        if not self.waypoint_bound:  
            
            # GET NEW HEADING:
            # Current waypoint:
            waypoint = self.waypoints[self.i]

            # Get relative angle to waypoint (degrees)
            angle_dif = get_turn_angle(self.x, self.y, self.heading, waypoint[0], waypoint[1])

            desired_heading = angle_dif + self.heading

            # Wait until we are oriented to move forward
            if abs(angle_dif) > 5:
                twist = Twist()
                twist.linear.x = 0.0 # Set linear v to 0
                twist.angular.z = desired_heading # Set desired heading
                self.heading_pub.publish(twist)

            else: # Once we are pointed at waypoint, move robot forward
                msg = Twist()
                msg.linear.x = self.straight_vel
                msg.angular.z = self.heading # Stay going straight, jut dont want this value to be none
                self.heading_pub.publish(msg)

                self.waypoint_bound = True # We are pointed at and headed to a waypoint, so can stop waypoint_bound loop for now

        # 3. BuG MODE!!
        if self.in_bug_mode:
            # WHEN FIRST ENTERIG BUGG MODE, WANT IT TO STOP, THEN TURN 90 DEGREES TO THE RIGHT
            # THEN TRACK THE RECENT DISTANCES, AND IF A FEW IN ROW ARE INF THEN TURN LEFT 90 DEGREES
            
           
            # Store the side distances
            self.side_dist_array.append(self.current_side_distance)

            # Check for if there is nothing to the side, set bool and heading for Bug Pt.B
            if len(self.side_dist_array) > 10:

                inf_count = self.side_dist_array.count(float('inf'))

                if inf_count > 7:

                    self.turning_left = True  # Set bool for the 'if elif else' below

                    # Clear the array so that turning left ^^ doesnt keep getting reset to true 
                    self.side_dist_array.clear()

                    # Also store heading at this moment so we don't keep adding 90 in the desired angle calc
                    self.left_clear_heading = self.heading


            # Bug Pt A. -  90 DEGREE TURN AT THE START
            if self.bug_stop_and_turn:  # Bug just started, stop and turn 90 deg to right
                # Keep looping this until we are stopped and turned to the right
                ninety_to_right = self.hitpoint_heading - 90 # Desired angle to the right

                angle_dif = get_smallest_angle(ninety_to_right,self.heading)

                # Make sure rover is stopped until at correct heading
                if abs(angle_dif) > 5:
                    move = Twist()
                    move.linear.x = 0.0 # Stop rover
                    move.angular.z = ninety_to_right

                    # Publish
                    self.heading_pub.publish(move)

                else: #When turned to the right spot, continue regular bug mode
                    self.bug_stop_and_turn = False

            # Bug Pt. B  - 90 DEGREE TURN TO LEFT IF NOTHING TO THE SIDE (ARRAY CHECKED AT TOP)
            elif self.turning_left:
                
                ninety_to_left = self.left_clear_heading - 90

                angle_dif = get_smallest_angle(ninety_to_left,self.heading)

                if abs(angle_dif) > 5:
                    move = Twist()
                    move.linear.x = 0.0 # Stop rover
                    move.angular.z = ninety_to_left

                    # Publish
                    self.heading_pub.publish(move)

                # After done tunring, done with this section for now
                else:
                    self.turning_left = False 
                
            # 3. Bug Pt. C - NORMAL BUGG CIRCUMSTANCES - PROPORTIONAL WALL FOLLOWING
            else:
                # Init twist message
                twist = Twist()

                # Use angle_adjuster to get the angle we should turn based on side distance
                angle, turn_bool = angle_adjuster(self.current_side_distance,self.ref_distance)
            
                # self.logger log position and turn angle (will test that function anyway)
                
                # Drive forward
                twist.linear.x = self.bug_velocity

                # Get angle to turn to
                desired_angle = angle + self.heading # Get absolute angle to turn to

                # Desired heading (NOT ACTUALLY ANGULAR V- just using twist msg to send all data to cmd node at once
                twist.angular.z = desired_angle

                self.heading_pub.publish(twist)

                # If back on m_line, stop bug mode, then need 
                # the heading for the waypoint again (do not increment waypoint though! same waypoint)
                if on_m_line((self.x,self.y),self.hitpoint,self.waypoints[self.i]):
                    # Back on m line, on other side of obstacle  
                    self.in_bug_mode = False  # End bug mode
                    self.waypoint_bound = False # Need to get the heading again

    # Lidar callback: Subs to the lidar data straight fro GZ
    def lidar_callback(self,msg):
        
        # msg should be in array format
        scan_array = msg

        # Get front and side distances
        front_distance = get_front_distance(scan_array)

        side_distance = get_left_distance(scan_array)

        # Set as current for timer callback
        self.current_side_distance = side_distance

        # Append to side distances array to track if we are around the corner
        self.side_dist_array.append(side_distance)

        # Only run this section for when we arent in bug mode and want to
        # see if we should be
        if not self.in_bug_mode:
            # Check if obstacle straight ahead
            if obstacle_ahead(front_distance):

                # If we see an obstacle, check if its worth starting bug mode
                startbug, hitpoint = start_bug_mode((self.x,self.y),self.waypoints[self.i],cutoff_distance=2.5)
                
                if startbug:
                    self.ref_distance = front_distance # Will want to keep side distance at this distance during ctrl
                    self.hitpoint = hitpoint  # Store hit point for m_line tracking
                    self.in_bug_mode = True  # Activate bug mode for timer callback bug section
                    self.bug_stop_and_turn = True # Set as true when first entering bug mode
                    self.hitpoint_heading = self.heading # Store current heading to track turning
                
                else: # Start_bug checks if its worth going around obstacle, if not just go to next waypoint
                    self.waypoint_bound = False

                    # update waypoint number
                    self.i +=1
            
        
    def flag_node_callback(self,msg):
        # Callback for flag search node
        self.flag_found = msg.data

    def gps_callback(self,msg):
        # Get the x,y position of the rover from the model states topic
        # This is a list of all models in the simulation, so we need to find our robot in it
        # The index of our robot is given by the name of the robot (in gazebo)
        # We can use this to get the x,y position of our robot
        self.x = msg.pose[msg.name.index("rover")].position.x
        self.y = msg.pose[msg.name.index("rover")].position.y

    def imu_callback(self,msg):
        quaternion = msg.pose[msg.name.index("rover")].orientation
        # Convert quaternion to yaw angle
        yaw_rad, yaw_deg = quaternion_to_yaw(quaternion)

        # Set as current heading (from positive x axis)
        self.heading = yaw_deg

# Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the node
    node = NavNode()

    #Spin
    rclpy.spin(node)

    #When done, shutdown
    rclpy.shutdown()