#!/usr/bin/env python3

# Cmd turning node
# Ian Spehar

import rclpy
import numpy as np
from rclpy.node import Node

# Velocity commands and IMU data
from geometry_msgs.msg import Twist, Quaternion

# (x,y), quaternion for yaw position from Gazebo:
from gazebo_msgs.msg import ModelState

from std_msgs.msg import Float32

# Functions
from head import quaternion_to_yaw, get_smallest_angle

class CommandNode(Node):
    
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('cmd_node')

        # SUBSCRIPTIONS (msg type,topic name,callback,queue size)
        # Desired heading from nav node
        self.heading_sub = self.create_subscription(Float32, 'heading_and_velocity',self.subscription_callback,10) 
        
        # Sub to imu
        self.imu_sub = self.create_subscription(ModelState, 'heading',self.imu_callback,10)
        # PUBLISHER
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer callback, this is where actual control will happen 
        self.timer = self.create_timer(0.1, self.timer_callback)

        #VARIABLES:
        self.heading = None # Current heading of the rover (degrees)
        self.desired_heading = None # Desired heading we get from the nav node
        self.linear_v = None
        self.angle_tolerance = 5 # Just need to be within 5 degrees

        self.get_logger().info('Cmd node ready')

    # Turns until heading within range of desired angle
    def timer_callback(self):

        # Check that we have data to use
        if self.desired_heading is not None and self.heading is not None:

            twist = Twist()

            # Get the angle difference (function calcs quickest route)
            angle_dif = get_smallest_angle(self.desired_heading, self.heading)

            # Turn until within 5 deg of desired angle
            if abs(angle_dif) > 5: # If we are not within 5 degrees of the desired angle
        
                # Turn depending on direction of quickest angle_dif
                if angle_dif > 0:
                    twist.angular.z = 1.0
                else:
                    twist.angular.z = -1.0

            else: # Desired angle reached
                twist.angular.z = 0.0 # Stop turning for now
            
            # Set linear velocity
            twist.linear.x = self.linear_v

            self.twist_pub.publish(twist) # Publish the twist message 


    # Store heading and linear v cmd from nav node
    def subscription_callback(self,msg):
        self.desired_heading = msg.angular.z  # Actually desired heading, just using twist msg type
        self.linear_v = msg.linear.x
       
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
    node = CommandNode()

    #Spin
    rclpy.spin(node)

    #When done, shutdown
    rclpy.shutdown()