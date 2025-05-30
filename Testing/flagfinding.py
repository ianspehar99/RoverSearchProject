
import cv2
import numpy as np



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
        
    # Flag not detected, returns an impossible position
    return flag_detected, (cx,cy)


img_bgr = cv2.imread('red_flag.png')

flag_color = 'red'
flag_detected, centroid = flag_detector(img_bgr,flag_color,area_threshold = 1000)
cx = centroid[0]
cy = centroid[1]

color = (0, 255, 0)
cv2.circle(img_bgr, (cx, cy), 6, color, -1)
cv2.putText(img_bgr, "Flag", (cx + 10, cy),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

# --- 4. Show image ---
cv2.imshow("Test Result", img_bgr)
cv2.waitKey(0)
cv2.destroyAllWindows()