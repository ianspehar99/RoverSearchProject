#!/usr/bin/env python3

# Main Navigation Node for Autonomous Search

# Subscribes to GPS, IMU, and Depth camera data
# Navigates robot to waypoints, and goes around obstacles when needed using bug mode
# Publishes cmd_vel and twists to the rover

# Ian Spehar

import rclpy
import numpy as np
import csv
from rclpy.node import Node

# Import srv for starting/stopping
from services_messages.srv import SendData

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
from head import get_turn_angle, reached_waypoint, quaternion_to_yaw
import camera_avoidance 
import goal_finder

class NavNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('main_nav_node')

        # SUBSCRIPTIONS (msg type,topic name,callback,queue size)
        # Camera depth image
        self.cam_sub = self.create_subscription(Image, 'depth_topic_name',self.camera_depth_callback,10) 
        # GPS and IMU data (For my project, simple x,y - for rover, will be actual gps)
        self.gps_sub = self.create_subscription(ModelState, '/gazebo/model_states',self.gps_callback,10) 
        
        # IMU data - This will be seperate on actual rover, for now the gps_sub gets x,y, and quat from gazebo
        #self.imu_sub = self.create_subscription(ModelState, 'heading',self.imu_callback,10)

        # PUBLISHER
        self.twist_pub = self.create_publisher(Twist, 'velocity',10)


        # Creates service  (type,ros2 call name,callback)
        self.service = self.create_service(SendData, 'startstop', self.service_callback)

        # TIMER
        freq =   10 #Publish at 10 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.timer_callback)

        #VARIABLES:
        self.waypoints = [(10,0),(0,10),(-10,0),(-3,-7),(4,0),(0,4),(-4,0)] # List of waypoints to navigate to
        self.i = 0  # Index to track which waypoint we are on
        self.x = 0  # Current x position of the rover
        self.y = 0  # Current y position of the rover
        self.heading = 0 # Current heading of the rover
        self.in_bug_mode = False # Flag to indicate if we are in bug mode
        self.waypoint_bound = False # Flag to indicate if goal waypoint is currently set
        self.straight_vel = 2 # Linear velocity
        self.bug_velocity = 1 # Bug velocity
        self.pixel_counts = {} # Pixel counts for each region from camera_avoidance
        self.hitpoint = ()  # Stores robot position at time of obstacle detected/ bug started

        self.get_logger().info('Nav node ready, waiting for service call...')

    # Timer callback: The main code that will loop continously
    def timer_callback(self):

        # 1. First, check if we are near the current goal waypoint (Default thresh at 0.5)
        # Function takes in (tuple of rover position) , (tuple of current waypoint coords)
        if reached_waypoint((self.x,self.y),self.waypoints[self.i]):
            self.i += 1 # Update waypoint index for next goal

            # Need new heading, change this so that if statement 2 is triggered
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
            _,angle = get_turn_angle(self.x, self.y, self.heading, waypoint[0], waypoint[1])

            desired_angle = angle[0] + self.heading # Get absolute angle to turn to

            # Turn until within 5 deg of desired angle
            while abs(self.heading - desired_angle) > 5: # If we are not within 5 degrees of the desired angle
                # Init twist message
                twist = Twist()
                twist.angular.z = 1.0
                self.velocity_pub.publish(twist) # Publish the twist message to start turning
            
            # Once done, stp turning, and start moving forward
            # Set angular velocity
            twist.angular.z = 0.0 # Stop turning

            # Set linear velocity for going straight (will maintain this v until overriden by other cmd)
            twist.linear.x = self.straight_vel # Set linear velocity
            self.velocity_pub.publish(twist) # Publish the twist message to start moving forward

            self.waypoint_bound = True # We are now headed to a waypoint

        # 3. Handle commands for if we are in bug mode
        if self.in_bug_mode:  

            # --- 1. Use get_position to see where you are relative to the obstacle and angle to turn
            angle, position = camera_avoidance.get_position(self.pixel_counts,500) 
            # self.logger log position and turn angle (will test that function anyway)
            
            # Driving commands (same code from main loop)
            desired_angle = angle + self.heading # Get absolute angle to turn to

            # Turn until within 5 deg of desired angle
            while abs(self.heading - desired_angle) > 5: # If we are not within 5 degrees of the desired angle
                # Init twist message
                twist = Twist()
                twist.angular.z = 1.0
                self.velocity_pub.publish(twist) # Publish the twist message to start turning

            # After each small turn, stop and then go forward
            twist.angular.z = 0.0
            twist.linear.x = self.bug_velocity


            # --- 2. Check if back on m_line using on_m_line

            # Takes in current x,y, self.hitpoint, current waypoint, threshold you can check the func ig
            # If back on m_line, stop bug mode self.in_bug_mode = False, then need 
            # the heading for the waypoint again (do not icrement waypoint though! same waypoint)
            if camera_avoidance.on_m_line((self.x,self.y),self.hitpoint,self.waypoints[self.i]):
                # Back on m line, on other side of obstacle - 
                self.in_bug_mode = False  #End bug mode
                self.waypoint_bound = False # Need to get the heading again

    # Camera callback: Every time we get a depth cam image 
    # (or every n times, can use count based throttling)
    def camera_depth_callback(self,msg):
 

        # RUN goal_finder.image_msg_to_np to convert image 
        depth_img = goal_finder.image_msg_to_np(msg) # Convert image to numpy array

        ## Run count_obstacles >> Output is self.pixel_counts (which is input to get_position in timer callback)
        self.pixel_counts = camera_avoidance.count_obstacles(depth_img, threshold=0.5) 

        # Check if obstacle straight ahead
        if camera_avoidance.is_obstacle_in_center(self.pixel_counts, sensitivity=1000):

            # If we see an obstacle in the center region, we check if we should start bug mode
            startbug, hitpoint = camera_avoidance.start_bug_mode((self.x,self.y),self.waypoints[self.i],cutoff_distance=2.5):
            
            if startbug:
                self.hitpoint = hitpoint
                self.in_bug_mode = True
            
            else: # Start_bug checks if its worth going around obstacle, if not just go to next waypoint
                self.waypoint_bound = False

                # update waypoint number
                self.i +=1
                

        ## Run is_obstacle_in_center (obstacle ahead?)
        #           If yes >> Run start_bug_mode (checks if its worth doing bug, returns bool)
        #                   If yes (start_bug mode = True), self.start_bug = True
        #                   If no (start_bug mode = False), skip to next waypoint > self.waypoint_bound = False
        
    def camera_flag_callback(self,msg):
        ######## Need t figure out how to do this, seperate node maybe? idk

    def gps_callback(self,msg):
        # Get the x,y position of the rover from the model states topic
        # This is a list of all models in the simulation, so we need to find our robot in it
        # The index of our robot is given by the name of the robot (in gazebo)
        # We can use this to get the x,y position of our robot
        self.x = msg.pose[msg.name.index("your_robot_name")].position.x
        self.y = msg.pose[msg.name.index("your_robot_name")].position.y

    def imu_callback(self,msg):
        quaternion = msg.pose[msg.name.index("your_robot_name")].orientation
        # Convert quaternion to yaw angle
        yaw_rad, yaw_deg = quaternion_to_yaw(quaternion)

        # Set as current heading (from positive x axis)
        self.heading = yaw_deg

    
    def service_callback(self, request, response):
        
        #This can just be very simple stop/start command or not i meannnn

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