from robots import *
import time
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import numpy as np
import random
from collections import Counter  

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
robot = Robot_OS(sim, DeviceNames.ROBOT_OS)

top_image_sensor = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

# HELPER FUNCTION
def show_image(image):
    plt.imshow(image)
    plt.show()

# Starts coppeliasim simulation if not done already
sim.startSimulation()
time.sleep(0.5)


# Definition of Color profiles for color detection. Each color is represented by a list of RGB tuples.
COLOR_PROFILES = {
    "wall_purple": [(45, 47, 58), (40, 42, 55)], 
    "goal_blue":   [(65, 75, 100), (50, 60, 90), (40, 50, 85), (30, 45, 75), 
                    (70, 85, 115), (55, 70, 95), (60, 80, 110),
                    (20, 30, 60),  (25, 35, 65), (15, 25, 50), 
                    (35, 50, 80),  (45, 60, 90), (30, 40, 70)], 
    "goal_red":    [(85, 15, 15), (70, 5, 5)],    
    "green":       [(0, 80, 0), (15, 95, 15), (25, 85, 25), (35, 92, 35), (10, 60, 10), (20, 70, 20)],    
    "brown":       [(45, 30, 15), (55, 35, 20), (40, 25, 10)],  
    "black":       [(0, 0, 0), (8, 8, 8), (12, 12, 12)],        
    "yellow":      [(90, 90, 0), (80, 80, 10), (75, 75, 5), (95, 95, 10), (85, 85, 15)],                 
    "orange":      [(85, 45, 15), (90, 50, 10), (95, 55, 5), (80, 40, 20), (75, 35, 10)],
    "floor_grey":  [(35, 35, 40)]                 
}

# A hormone dictionary that is used to track the robots internal state and goals for decision making.
HORMONES = {
    "avoid_wall": 0.0,
    "green_drive": 0.0,
    "black_drive": 0.0,
    "rest": 0.0
}

# A dictionary with the most hormone decay rates for each hormone.
DECAY_RATES = {
    "avoid_wall": 0.5,
    "green_drive": 0.002,
    "black_drive": 0.002,
    "rest": 0.0
}

# --- Variables ---
BASE_SPEED = 1.8       
carrying_block = False
carrying_green = False  

BUFFER_SIZE = 5
top_color_buffer = []
small_color_buffer = []
bottom_color_buffer = []

def get_smoothed_color(color_buffer, new_color):
    """
    Get the most common color in the buffer. 
    """
    color_buffer.append(new_color) # Add the new color to the buffer
    if len(color_buffer) > BUFFER_SIZE: # If the buffer is too big, remove the oldest color
        color_buffer.pop(0)
    return Counter(color_buffer).most_common(1)[0][0] # return the most common color in the buffer

def color_distance(c1, c2):
    """
    Calculate the Euclidean distance between two RGB colors.
    """
    return np.sqrt((c1[0] - c2[0])**2 + (c1[1] - c2[1])**2 + (c1[2] - c2[2])**2)

def normalize_rgb(raw_rgb):
    """
    Normalize the RGB values to a 0-100 scale
    """
    return (raw_rgb[0] / 255 * 100, raw_rgb[1] / 255 * 100, raw_rgb[2] / 255 * 100)
    
def get_horizontal_image(image):
    """
    Extract a horizontal slice of the image (between 25% and 35% of the height) and return the average RGB color.c
    """
    height, _, _ = np.array(image).shape
    horizon_slice = image[int(height * 0.25):int(height * 0.35), :]
    avg_rgb = horizon_slice.mean(axis=(0, 1))
    return normalize_rgb(avg_rgb)

def get_central_image(image):
    """
    Extract a central square slice of the image and return the average RGB color.
    """
    height, width, _ = np.array(image).shape
    central_slice = image[height//2-3:height//2+3, width//2-3:width//2+3]
    avg_rgb = central_slice.mean(axis=(0, 1))
    return normalize_rgb(avg_rgb)

def navigate_to_goal(image, target_color):
    """
    Analyze the top image to find if the target color is present and return the direction of the goal.
    """
    img_np = np.array(image)
    height, width, _ = img_np.shape
    
    matching_cols = [] # List where we store the column indices if the pixel color matches the target color
    for row in range(0, min(5, height)):
        for col in range(width):
            pixel = img_np[row, col]
            avg_rgb = normalize_rgb(pixel)
            
            if get_main_color_rgb(avg_rgb) == target_color:
                matching_cols.append(col)
                
    # If no pixels where found that match the color we return "not_found"
    if not matching_cols:
        return "not_found"
        
    # Get the average color column and compare it to the middle of the image to get the direction of the goal    
    avg_col = np.mean(matching_cols)
    center = width / 2
    
    if abs(avg_col - center) < 4:  
        return "center"
    elif avg_col < center:
        return "left"
    else:
        return "right"

def get_bottom_image(image):
    """
    Extract the bottom 40% of the image and return the average RGB color.
    """
    height, _, _ = np.array(image).shape
    bottom = image[int(height * 0.6):, :]
    avg_rgb = bottom.mean(axis=(0, 1))
    return normalize_rgb(avg_rgb)

def block_loss(prev_image, current_image, target_color):
    """
    Check if the target color was lost between the previous and current image to detect if the block was dropped.
    """
    if prev_image is None or current_image is None:
        return False
    prev_image_b = get_bottom_image(prev_image)
    current_image_b = get_bottom_image(current_image)
    prev_color = get_main_color_rgb(prev_image_b)
    current_color = get_main_color_rgb(current_image_b)
    if prev_color == target_color and current_color != target_color:
        return True
    return False
    
def get_main_color_rgb(rgb):
    """
    Get the main color of the given RGB of the image by comparing it to the color profiles.
    """
    if np.sum(rgb) < 10:
        return "black"
    
    closest_color = "unknown"
    threshold = 25
    
    # Compare every color profile to the given RGB and return the name of the closest color.
    for color_name, profiles in COLOR_PROFILES.items():
        for profile in profiles:
            dist = color_distance(rgb, profile)
            if dist < threshold:
                threshold = dist
                closest_color = color_name
                
    return "unknown" if closest_color == "floor_grey" else closest_color

def zone_contains_color(zone, target_color):
    """
    Checks if a given zone of the image contains the target color using checkerboard sampling and return the number of matches.
    """
    matches = 0
    h, w, _ = zone.shape

    # Using a checkerboard pattern to sample a subset of pixels in the zone.
    for y in range(0, h, 2):
        for x in range(0, w, 2):
            rgb = normalize_rgb(zone[y, x])
            if get_main_color_rgb(rgb) == target_color:
                matches += 1

    return matches

def turn_towards_cube(image, target_colors):
    """
    Analyze the image zones to find if the target color is present and return the direction of the cube.
    """
    h, w, _ = image.shape

    zones = {
        "left": image[h//3:h, :w//3],
        "center": image[h//3:h, w//3:2*w//3],
        "right": image[h//3:h, 2*w//3:]
    }

    best_zone = "none"
    best_score = 0

    # Checks every zone and returns the zone with the the highest number of matches for the target color.
    for zone_name, zone_img in zones.items():
        score = 0
        for color in target_colors:
            score += zone_contains_color(zone_img, color)

        if score > best_score:
            best_score = score
            best_zone = zone_name
            
    if best_score > 5:
        return best_zone

    return "none"

def drive(left = BASE_SPEED, right = BASE_SPEED):
    """
    Simple drive forward function.
    """
    left_motor.run(left)
    right_motor.run(right)
    
def stop():
    """
    Stops the robot
    """
    left_motor.run(0)
    right_motor.run(0)
    
def update_hormones(dist, battery_level, top_color, small_color, bottom_color):
    """
    Updates the hormone levels based on the sensor readings and decay rates
    """
    global HORMONES
    global carrying_green
    global carrying_block
    
    # Hormones decay over time but never reach negative values.
    for hormone in HORMONES:
        HORMONES[hormone] = max(0.0, HORMONES[hormone] - DECAY_RATES[hormone])
        
    # The robot should avoid walls so to increase the hormone
    if dist < 0.25 and (top_color == "wall_purple" or small_color == "wall_purple") and small_color not in ["green", "black", "brown"] and top_color not in ["green", "black", "brown"]:
        HORMONES["avoid_wall"] = 1.0
                
    # To avoid falling in the bin if the sensors show the black background
    if not carrying_block and not carrying_green and small_color == "black" and top_color == "black" and dist > 0.22:
        HORMONES["avoid_wall"] = 1.0
        
    # If the robot is carrying a block and is close enough to the goal the robot should turn after dropping the block.
    if HORMONES["green_drive"] <= 0.2 and (top_color == "goal_blue" or small_color == "goal_blue") and dist < 0.28:
        HORMONES["avoid_wall"] = 1.0
    if HORMONES["black_drive"] <= 0.2 and (top_color == "goal_red" or small_color == "goal_red") and dist < 0.28:
        HORMONES["avoid_wall"] = 1.0
        
    # Update the resting hormone when based on the battery level when the battery is low.
    HORMONES["rest"] = max(0.0, 1.0 - battery_level / 0.2) if battery_level < 0.2 else 0.0
    
    # For testing:
    #HORMONES["rest"] = 1 if battery_level < 0.99 else 0.0
    
    # If the robot detects green or black blocks the robot should increase the specific hormones.
    if (small_color == "green" or bottom_color == "green") and dist < 0.17:
        HORMONES["green_drive"] = 1.0
        carrying_green = True 
    
    if small_color == "black" and dist < 0.2:
        HORMONES["black_drive"] = 1.0
        carrying_block = True
        
prev_image = None 

while True:
    # Get the sensor readings and process the images
    top_image_sensor._update_image()
    small_image_sensor._update_image()
    
    top_image = top_image_sensor.get_image()
    small_image = small_image_sensor.get_image()
    bottom_image = get_bottom_image(small_image)
    
    raw_top_color = get_main_color_rgb(get_horizontal_image(top_image))
    raw_small_color = get_main_color_rgb(get_central_image(small_image))
    raw_bottom_color = get_main_color_rgb(bottom_image)
    
    top_color = get_smoothed_color(top_color_buffer, raw_top_color)
    small_color = get_smoothed_color(small_color_buffer, raw_small_color)
    bottom_color = get_smoothed_color(bottom_color_buffer, raw_bottom_color)
    
    dist = robot.get_sonar_sensor()
    battery_level = robot.get_battery()
    
    update_hormones(dist, battery_level, top_color, small_color, bottom_color)
    
    # Printing the current state of the robot
    print(f"Hormones: {HORMONES}", end=" | ")
    print(f"Top Color: {top_color}, Small Color: {small_color}, Distance: {dist:.2f}, Battery: {battery_level:.2f} | Carrying Block: {carrying_block}, Carrying Green: {carrying_green}")
    
    # LAYER 5: AVOID WALL / HAZARDS
    if HORMONES["avoid_wall"] > 0.5:
        # The robot should back of when too close to the wall
        drive(-BASE_SPEED, -BASE_SPEED)
        time.sleep(0.4)
        # Turn in a random direction away from the wall
        turn_dir = 1 if random.random() > 0.5 else -1
        drive(-BASE_SPEED * 1.5 * turn_dir, BASE_SPEED * 1.5 * turn_dir)
        time.sleep(random.uniform(0.4, 0.7))
        HORMONES["avoid_wall"] = 0.0 
        
        top_color_buffer.clear()
        small_color_buffer.clear()
        bottom_color_buffer.clear()
        
        prev_image = small_image.copy()
        continue
    
    # LAYER 4: BATTERY / CHARGING STATION NAVIGATION
    elif HORMONES["rest"] > 0.7:
        # Check if yellow is detected underneath the robot
        if small_color == "yellow" or bottom_color == "yellow":
            drive(BASE_SPEED, BASE_SPEED)
            time.sleep(1.5)
            stop()
            print("--- CHARGING STATION REACHED ---")
            # Stop the robot to charge the battery
            time.sleep(4.0) 
            
            top_color_buffer.clear()
            small_color_buffer.clear()
            bottom_color_buffer.clear()
            
            prev_image = small_image.copy()
            continue
            
        # If not on the pad, seek the orange beacon profile
        charge_direction = navigate_to_goal(top_image, "orange")
        if charge_direction == "not_found":
            # Rotate on the spot to find the charging station
            drive(-BASE_SPEED * 0.6, BASE_SPEED * 0.6) 
        elif charge_direction == "left":
            drive(BASE_SPEED * 0.3, BASE_SPEED * 1.1)
        elif charge_direction == "right":
            drive(BASE_SPEED * 1.1, BASE_SPEED * 0.3)
        else: 
            drive(BASE_SPEED, BASE_SPEED)
            
        time.sleep(0.05)
        prev_image = small_image.copy()
        continue
    
    # LAYER 3: GOAL SEEKING (GREEN)
    elif HORMONES["green_drive"] > 0.2:
        # If we loss the block we need to check if we are close to the goal or lost it on the map
        if carrying_green and block_loss(prev_image, small_image, "green"):
            if (top_color == "goal_blue" or small_color == "goal_blue") and dist < 0.35:
                # When dropping it in the bin we should back off and avoid falling into the bin
                carrying_green = False
                HORMONES["green_drive"] = 0.0
                drive(-BASE_SPEED, -BASE_SPEED)
                time.sleep(1.2)
                drive(-BASE_SPEED, BASE_SPEED)
                time.sleep(0.5)
                
                top_color_buffer.clear()
                small_color_buffer.clear()
                bottom_color_buffer.clear()
                
            else:
                # If we lost the block on the map we should reset the hormones
                carrying_green = False
                HORMONES["green_drive"] = 0.0
            prev_image = small_image.copy()
            continue
    
        if carrying_green:
            # If we have the block we should look for the goal
            goal_direction = navigate_to_goal(top_image, "goal_blue")
            
            PUSH_SPEED = BASE_SPEED * 0.65
            if goal_direction == "not_found":
                drive(PUSH_SPEED * 0.7, PUSH_SPEED * 1.2) 
            elif goal_direction == "left":
                drive(PUSH_SPEED * 0.4, PUSH_SPEED * 1.1)
            elif goal_direction == "right":
                drive(PUSH_SPEED * 1.1, PUSH_SPEED * 0.4)
            else: 
                # Add some noise to make it more robost and prevent getting stuck.
                noise = np.random.normal(0, 0.1)
                drive(PUSH_SPEED + noise, PUSH_SPEED - noise)
            time.sleep(0.05)
            prev_image = small_image.copy()
            continue
            
        else:
            # If we dont have a cube we should look for cubes and move towards them
            direction = turn_towards_cube(small_image, ["green"])
            if direction == "left":
                drive(BASE_SPEED * 0.1, BASE_SPEED * 0.4)
            elif direction == "right":
                drive(BASE_SPEED * 0.4, BASE_SPEED * 0.1)
            elif direction == "center":
                drive(BASE_SPEED * 0.6, BASE_SPEED * 0.6)
            else:
                drive(-BASE_SPEED * 0.25, BASE_SPEED * 0.25)
            time.sleep(0.05)
            prev_image = small_image.copy()
            continue
    
    # LAYER 3: GOAL SEEKING (BLACK)
    elif HORMONES["black_drive"] > 0.2:
        # If we lost a block we need to check if we are close to the goal or lost it on the map
        if carrying_block and (block_loss(prev_image, small_image, "black") or block_loss(prev_image, small_image, "brown")):
            # When the robot drops the block in the bin we need to back off to avoid falling into the bind
            if (top_color == "goal_red" or small_color == "goal_red") and dist < 0.3:
                carrying_block = False
                HORMONES["black_drive"] = 0.0
                drive(-BASE_SPEED, -BASE_SPEED)
                time.sleep(1.2)
                drive(-BASE_SPEED, BASE_SPEED)
                time.sleep(0.5)
                
                top_color_buffer.clear()
                small_color_buffer.clear()
                bottom_color_buffer.clear()
                
                prev_image = small_image.copy()
                continue
        
        # If we have the block we should look for the goal.
        goal_direction = navigate_to_goal(top_image, "goal_red")
        if goal_direction == "not_found":
            drive(BASE_SPEED * 0.7, BASE_SPEED * 1.2)  
            time.sleep(0.05)
        elif goal_direction == "left":
            drive(BASE_SPEED * 0.4, BASE_SPEED * 1.1)   
            time.sleep(0.05)
        elif goal_direction == "right":
            drive(BASE_SPEED * 1.1, BASE_SPEED * 0.4)   
            time.sleep(0.05)
        else:  
            noise = np.random.normal(0, 0.1)
            left_speed = BASE_SPEED + noise
            right_speed = BASE_SPEED - noise
            drive(left_speed, right_speed)
            time.sleep(0.1)
            
        prev_image = small_image.copy()
        continue
    
    # LAYER 2: INTERACTION with brown blocks
    elif small_color == "brown" and dist < 0.22:
        stop()
        robot.compress() # compress
        time.sleep(1)
        
        drive(BASE_SPEED, BASE_SPEED)
        time.sleep(0.4) # Drive forward to have enough space to turn
        stop()
        time.sleep(0.1)
        
        PIVOT_SPEED = 1.55 
        drive(-PIVOT_SPEED, PIVOT_SPEED) # Rotate 180 degrees 
        time.sleep(1.06)
        stop()
        time.sleep(0.1)
        
        drive(BASE_SPEED, BASE_SPEED) # Drive forward to get the block in the claws
        time.sleep(0.75)
            
        top_color_buffer.clear()
        small_color_buffer.clear()
        bottom_color_buffer.clear()
        
        prev_image = small_image.copy()
        continue
    
    # LAYER 1: FIND BLOCKS
    if (small_color in ["black", "brown", "green"] or bottom_color in ["black", "brown", "green"]) and dist < 0.24:
        drive(BASE_SPEED * 1.1, BASE_SPEED * 1.1)
        time.sleep(0.1)
    direction = turn_towards_cube(small_image, ["green", "black", "brown"])
    if direction != "none":
        if direction == "left":
            drive(BASE_SPEED * 0.4, BASE_SPEED * 1.1)
        elif direction == "right":
            drive(BASE_SPEED * 1.1, BASE_SPEED * 0.4)
        elif direction == "center":
            drive(BASE_SPEED, BASE_SPEED)
        prev_image = small_image.copy()
        continue
    
    # DEFAULT (LAYER 0): EXPLORE
    else:
        # Random movement to explore the environment.
        r = random.random()
        if r < 0.15:
            drive(BASE_SPEED * 0.4, BASE_SPEED * 1.3)  
        elif r > 0.85:
            drive(BASE_SPEED * 1.3, BASE_SPEED * 0.4)  
        else:
            drive(BASE_SPEED * 1.1, BASE_SPEED * 1.1)  
            
    prev_image = small_image.copy()
    time.sleep(0.05) 