#!/usr/bin/env python3

# Flag search node
# Subs to gazebo rgb cam output, publishes found_flag bool to nav node
# Ian Spehar

import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Bool

from sensor_msgs.msg import Image

from flag_finder import flag_detector

from cv_bridge import CvBridge

class FlagNode(Node):
    
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('flag_node')

        # SUBSCRIPTIONS (msg type,topic name,callback,queue size)

        # Gazebo camera output
        self.cam_sub = self.create_subscription(Image, '<gazebo/ros/cam/topic/bridge>',self.subscription_callback,10) 
        
        # PUBLISHER:
        self.flag_found_pub = self.create_publisher(Bool, 'flag_found', 10)

        # Timer callback:
        self.timer = self.create_timer(0.1, self.timer_callback)

        #VARIABLES:
        self.flag_color = 'red' # Set flag color
        self.image = None   # Store image data here
        self.flag_found_array = []  # Track our flag_found bools, want to make sure

        # Init cv bridge
        self.bridge = CvBridge()


        self.get_logger().info('Flag node ready')

    # Process images to see if flag is there
    def timer_callback(self):

        if self.image is not None:

            image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')

            # Retunr flag detection bool, and location for the hell of it
            flag_detected, flag_location = flag_detector(image,self.flag_color,area_threshold = 1000)

            # Track in array
            self.flag_found_array.append(flag_detected)

            # Check every 5 times
            if len(self.flag_found_array) > 5:
                
                # Get how many of last 5 show flag 
                flag_found_count = self.flag_found_array.count(True)

                # Threshold to see if we should consider the flag found
                if flag_found_count > 4:

                    # If yes, send bool to nav node to signify mission complete
                    msg = Bool()
                    msg.data = True
                    self.flag_found_pub.publish(msg)

                else: # If not, clear array
                    self.flag_found_array.clear()

    # Get image data from gazebo
    def subscription_callback(self,msg):
        self.image = msg
        
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the node
    node = FlagNode()

    #Spin
    rclpy.spin(node)

    #When done, shutdown
    rclpy.shutdown()