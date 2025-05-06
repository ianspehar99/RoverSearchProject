#!/usr/bin/env python3

# Main Navigation Node for Autonomous Search

# Subscribes to GPS, IMU, and Depth camera data
# Navigates robot to waypoints, and goes around obstacles when needed using bug mode
# Publishes cmd_vel and twists to the rover

# Ian Spehar

# 
import rclpy
import numpy as np
import csv
from rclpy.node import Node

# Import srv for starting/stopping
from services_messages.srv import SendData

# Image datatype
from sensor_msgs.msg import Image

# Velocity commands
from geometry_msgs.msg import Twist


# x-y position from Gazebo:
from gazebo_msgs.msg import ModelStates
# Access later with msg.pose[index].position.x
#                   msg.pose[index].position.y
# index = msg.name.index("your_robot_name")


class NavNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('main_nav_node')

        # SUBSCRIPTIONS (msg type,topic name,callback,queue size)
        # Camera depth image
        self.cam_sub = self.create_subscription(Image, 'depth_topic_name',self.camera_callback,10) 
        # GPS data (For my project, simple x,y - for rover, will be actual gps)
        self.gps_sub = self.create_subscription(ModelStates, 'coordinates',self.gps_callback,10) 

        # PUBLISHER
        self.avg_latency = self.create_publisher(Twist, 'velocity',10)

        # Creates service  (type,ros2 call name,callback)
        self.service = self.create_service(SendData, 'startstop', self.service_callback)

        # TIMER
        freq =   10 #Publish at 10 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.timer_callback)

        #VARIABLES:
        #
        # Gonna be a fuck ton of these
        #
        self.get_logger().info('Nav node ready, waiting for service call...')

    # Timer callback: The main code that will loop continously
    def timer_callback(self):
        
        # Main code here
        x = 1

    # Camera callback: Every time we get a depth cam image 
    # (or every n times, can use count based throttling)
    def camera_callback(self,msg):
        depth = msg.data #Idk if this is right
        
        ## Run count_obstacles >> Output is self.counts (which is input to get_position in timer callback)

        ## Run is_obstacle_in_center (obstacle ahead?)
        #           If yes >> Run start_bug_mode (checks if its worth doing bug, returns bool)
        #                   If yes (start_bug mode = True), self.start_bug = True
        #                   If no (start_bug mode = False), skip to next waypoint > self.waypoint_bound = False
        

    def gps_callback(self,msg):
        # Get the x,y position of the rover from the model states topic
        # This is a list of all models in the simulation, so we need to find our robot in it
        # The index of our robot is given by the name of the robot (in gazebo)
        # We can use this to get the x,y position of our robot
        self.x = msg.pose[msg.name.index("your_robot_name")].position.x
        self.y = msg.pose[msg.name.index("your_robot_name")].position.y

    
    def service_callback(self, request, response):
        
        #This can just be very simple stop/start command

# Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the node
    node = ReceiveNode()

    #Spin
    rclpy.spin(node)

    #When done, shutdown
    rclpy.shutdown()