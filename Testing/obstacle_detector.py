import cv2
import numpy as np

def depth_target_detector(depth_img, min_depth, max_depth,img_center_fraction = 1/3, area_threshold=1000, visualize =  False):
    """
    Inputs
    - depth_img: array of depth values (in mm or meters depending on the camera)
    - min/max_depth: define range where we consider the obstacle to be too close
    - area_threshold: min contour area to count as obstacle

    Returns:
    - (cx,cy) of obstacle, obstacle_centered bool, obstacle_ahead, width
    """

    # Define the range in the middle where we consider the object to be in the way
    height, width = depth_img.shape
    edge_fraction = (1 - img_center_fraction)/2
    center_band = (width*edge_fraction,width*(1-edge_fraction))

    # Create a binary mask for pixels in depth threshold

    # FOR REAL ONE, FIND WHERE PIXELS ABOVE THRESHOLD
    # mask = cv2.inRange(depth_img, min_depth, max_depth)

    # FOR TESTING, DO PIXELS INSTEAD
    mask = cv2.inRange(depth_img,0,100)

    # Clean the mask with morphological operations
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    # Find contours 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy = -1, -1  # Default values
    max_area = 0
    obstacle_centered = False
    obstacle_ahead = False

    biggest_contour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > area_threshold and area> max_area:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            max_area = area
            biggest_contour = cnt

    if biggest_contour is not None:
        print("Contour found")
        obstacle_ahead = True
        if center_band[0] <= cx <= center_band[1]:
            print("Obstacle in center")
            obstacle_centered = True
            #centroid_depth = float(depth_img[cy, cx])
        
    # OPTIONAL: Run the visulizer if visualize set to true
    if visualize:
        visualizer(depth_img,cx,cy,center_band,obstacle_ahead,obstacle_centered,height)

    return (cx, cy), obstacle_centered, obstacle_ahead, width



# COULD CREATE A SPEPERATE FUNCTION TO NEST WITHIN THIS ONE^^^, THAT SAVES THE IMAGE AND ADDS THE CENTER SECTION OUTINE, AND THE CENTROID
def visualizer(img,cx,cy,center_band,obstacle_ahead,obstacle_centered,height):
    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    left_line_x = round(center_band[0])
    right_line_x = round(center_band[1])
    # Draw lines for center band
    cv2.line(img_bgr,(left_line_x,0),(left_line_x,height) , (0, 0, 255),2)
    cv2.line(img_bgr,(right_line_x,0),(right_line_x,height),(0, 0, 255),2)

    if obstacle_ahead:
        color = (0, 255, 0) if obstacle_centered else (0, 255, 255)
        cv2.circle(img_bgr, (cx, cy), 6, color, -1)
        cv2.putText(img_bgr, "Obstacle", (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # --- 4. Show image ---
    cv2.imshow("Test Result", img_bgr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# --- 1. Create a synthetic test image ---
img = np.full((480, 640), 255, dtype=np.uint8)  # start with a white image

# Draw a black blob in the center to simulate an obstacle
cv2.circle(img, (400, 240), 50, 50, -1)  # value 50 = "close" in fake depth


# --- 2. Run detector ---
(centroid_x, centroid_y), obstacle_ahead, obstacle_centered, _ = depth_target_detector(img, 0, 100,visualize = True)



