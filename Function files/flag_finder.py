import cv2
import numpy as np

from sensor_msgs.msg import Image


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

    flag_detected = False
    for cnt in contours:
        if cv2.contourArea(cnt) > area_threshold:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            flag_detected = True
        
    # If flag not detected, returns Flase and an impossible position
    return flag_detected, (cx,cy)




