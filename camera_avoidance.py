import cv2
import numpy as np

# This script simulates a depth image from an online image, pretending tht its the output of a depth camera, where pixels
# represent distances to obstacles. It then counts the number of obstacle pixels in three regions: left, center, and right
sensitivity = 1000 # Min pixels needed in middle box to register an obstace


# Get the number of obstacle pixels in three regions: left, center, and right
def count_obstacles(depth_img, threshold=0.5):
    h, w = depth_img.shape
    y1, y2 = int(h * 0.4), int(h * 0.6)

    # Define box width 
    box_width = int(w * 0.2)

    # Define th
    boxes = {
        "left":   (y1, y2, 0, box_width),
        "center": (y1, y2, w // 2 - box_width // 2, w // 2 + box_width // 2),
        "right":  (y1, y2, w - box_width, w),
    }

    counts = {}
    for region, (r1, r2, c1, c2) in boxes.items():
        region_data = depth_img[r1:r2, c1:c2]
        mask = (region_data > 0) & (region_data < threshold)
        counts[region] = int(np.sum(mask))

    return counts


# This function takes the output "obstacle_counts" from the count_obstacles function 
# and checks if the number of obstacle pixels in the center region exceeds the 
# sensitivity threshold, meaning that we found an obstacle and have to start steering

# After this is put into effect, based on the boolean, set an initial angle to steer towards the right
# and enter bug mode
def is_obstacle_in_center(obstacle_counts, sensitivity):
    """
    Returns True if the number of obstacle pixels in the center region exceeds the given sensitivity threshold.
    """
    return obstacle_counts.get("center", 0) > sensitivity


# Makes decision to start bug mode when we see an obstacle, see how far we are from the waypoint, 
# and if we are close enough to it already, we can skip to next waypoint
# Take in bool from is_obstacle_in_center, and the current position of the rover, and the waypoint

def start_bug_mode(current_position, waypoint,cutoff_distance=2.5):
    xc = current_position[0]
    yc = current_position[1]
    xw = waypoint[0]
    yw = waypoint[1]
    distance = ((xc - xw) ** 2 + (yc - yw) ** 2) ** 0.5

    return distance < cutoff_distance



# using pixels in l,r,middle boxes, get position of object, then use to set adjustment angle
# Assume obstacle in left third (where we want it) is position 0
def get_position(counts, dif_sensitivity):

        Left = counts["left"]
        Middle = counts["center"]
        Right = counts["right"]

        # Get differences between boxes
        dif1 = Middle - Left
        dif2 = Right - Middle
        position = None

        # Logic to determine position of object based on pixel counts
        # First cond: Middle box significantly more object pixels than others
        if abs(dif1) > dif_sensitivity and abs(dif2) > dif_sensitivity and dif1 < 0:
            print("Obstacle straight ahead")
            position = 0.5
        elif abs(dif1) > dif_sensitivity and abs(dif2) < dif_sensitivity:
            if dif1 > 0:
                print("Obstacle at edge case to the right")
                position = 0.75
            elif dif1 < 0:
                print("Obstacle to the left") #This is the position that you want!!
                position = 0
        elif abs(dif1) < dif_sensitivity and abs(dif2) > dif_sensitivity:
            if dif2 > 0:
                print("obstacle to the right")
                position = 1
            elif dif2 < 0:
                print("Object at edge case to the left")
                position = 0.25
        else:
            position = 0
        
        # Now we got the relative position, use to set adjustment angle:
        angle = position * 25

        return position, angle

# This function is for when we are ging around obstacle, going to be checking for when we reach the m-line
# and can continue to the waypoint. Returns true or false
def on_m_line(current_position, hit_point,waypoint,distance_threshhold = 1,angle_threshold=0.1):
    # Get the waypoint vector
    dx1 = waypoint[0] - hit_point[0]
    dy2 = waypoint[1] - hit_point[1]

    # Get the current position vector
    dx2 = current_position[0] - hit_point[0]
    dy2 = current_position[1] - hit_point[1]

    # Calc angle between the two vectors
    dot_product = dx1 * dx2 + dy2 * dy2
    mag1 = (dx1 ** 2 + dy2 ** 2) ** 0.5
    mag2 = (dx2 ** 2 + dy2 ** 2) ** 0.5
    angle = dot_product / (mag1 * mag2)
    angle = np.arccos(angle)  # Radias

    distance_from_hit_point = ((current_position[0] - hit_point[0]) ** 2 + (current_position[1] - hit_point[1]) ** 2) ** 0.5

    if angle < angle_threshold and distance_from_hit_point > distance_threshhold:
        # Checks if our current vectr is close to the waypoint vector, and if we are sufficently far from og hit point (wont trigger immediately)
        return True, distance_from_hit_point, angle
    else:
        return False, distance_from_hit_point, angle
    

    

#_______________________________________________________________________________________
#For this its obviously going to be opposite because you want to steer away from obstacle
#actually the point you want it at is only in the left box

# Load grayscale image
gray_img = cv2.imread('doggo.png', cv2.IMREAD_GRAYSCALE)
if gray_img is None:
    raise FileNotFoundError("Image not found or cannot be loaded")

# Simulate depth
min_depth, max_depth = 0.2, 4.0
depth_meters = min_depth + (gray_img.astype(np.float32) / 255.0) * (max_depth - min_depth)

# Count obstacle pixels
obstacle_counts, regions = count_obstacles(depth_meters)
print("Obstacle counts:", obstacle_counts)

# Visualize
vis = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
colors = {"left": (255, 0, 0), "center": (0, 255, 0), "right": (0, 0, 255)}

for region, (r1, r2, c1, c2) in regions.items():
    color = colors[region]
    cv2.rectangle(vis, (c1, r1), (c2, r2), color, 2)

cv2.imshow("Obstacle Regions (Pixel Counts)", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()
