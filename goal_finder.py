# These functions look at camera images and spots the flag 


# WE ALSO NEED TO DESIGN A FUNCTON FOR THIS, START FLAG MODE 
# WHERE WE GO TO THE FLAG AUTOMATICALLY. JUST DO SAME SHIT AS WITH LINE FOLLOWING
# WITH THE CAMERA WITH RC CARS, IDENTIFY THE BOX WITH HIGH CONCENTRATION OF <COLOR> PIXELS

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# CONVERT ROS2 IMAGE DATA TYPE INTO A NUMPY/CV2 TYPE:

def image_msg_to_np(img_msg):
    bridge = CvBridge()  # Bridge instance
    #LOWKEY MIGHT NEED TO SET ENCODING TO 'bgr8'
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
     
    return cv_image

# If flag detected, returns cx,cy
def flag_detector(img,flag_color,area_threshold = 1000):
    # Takes in cv2 img and flag color
    color_range = {
    'red': [(20, 150, 150), (255, 200, 200)],
    'green': [(20, 100, 120), (255, 150, 160)],
    'blue': [(0, 140, 130), (255, 170, 160)],
}   
    (lower,upper) = color_range[flag_color] # Get color range from flag color input

    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
   
    mask = cv2.inRange(img_lab, np.array(lower), np.array(upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Placeholder vals for if not detected (just so we can return something)
    cx = -1
    cy = -1
    for cnt in contours:
        if cv2.contourArea(cnt) > area_threshold:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # Flag detected, flag position
            return True , (cx, cy)
        
    # Flag not detected, returns an impossible position
    return False, (cx,cy)