from picar import PiCar, test, configure
import time
import argparse
import sys
import cv2
import numpy as np
import math

# Allows the PiCar to safely take images and prevents the script from crashing 
def get_safe_image(car):
    try:
        return car.get_image()
    except Exception: 
        return None

# Analyzes an image to determine which color is the most dominant.
def get_dominant_color(img, debug=False):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the ranges for green and yellow 
    mask_green = cv2.inRange(hsv, (45, 100, 50), (85, 255, 255))
    mask_yellow = cv2.inRange(hsv, (15, 100, 70), (40, 255, 255))

    # Since red sits on the edges of the HSV range, two masks are needed to complete the mask. 
    mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
    mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Counts how many pixels are in each filter 
    green_pixels = cv2.countNonZero(mask_green)
    yellow_pixels = cv2.countNonZero(mask_yellow)
    red_pixels = cv2.countNonZero(mask_red)
    
    if debug:
        cv2.imwrite('RedMask.jpg', mask_red)
        cv2.imwrite('YellowMask.jpg', mask_yellow)
        cv2.imwrite('GreenMask.jpg', mask_green)
        

    MIN_THRESHOLD = 500 

    color_counts = {
        "Red": red_pixels,
        "Yellow": yellow_pixels,
        "Green": green_pixels
    }

    # Finds which color is the most dominant
    dominant_color = max(color_counts, key=color_counts.get)
    max_pixels = color_counts[dominant_color]

    # Only return a color if it beats the 500 pixel threshold, otherwise return none 
    if max_pixels > MIN_THRESHOLD:
        return dominant_color
    else:
        return "None"


# Argument Parsing 
parser = argparse.ArgumentParser(description='Help')
parser.add_argument('--tim', action='store', type=int, default=15, help='Time for program to run in seconds')
parser.add_argument('--delay', action='store', type=float, default=0.05, help='Delay between readings')
parser.add_argument("--debug", action="store_true", help="Print debug info to console")
parser.add_argument('--delta', action='store', type=float, default=0.5, help='Change in Shift')
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
parser.add_argument('--Kp_steer', action='store', type=float, default=0.2)
parser.add_argument('--Kd_steer', action='store', type=float, default=0.0)

args = parser.parse_args()

START_TIME = time.time()

# Initial state and color 
state = "Detecting"
color = "None" 

time_arr = []
dist_arr = []
duty_arr = []
rps_arr = []

steer_mid = 0
swivel_mid = 0

# Initialize PiCar 
car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_swivel_servo(swivel_mid)
car.set_steer_servo(steer_mid)
car.set_nod_servo(0) 
time.sleep(0)

# Accelerometer parameters 
target_heading = 0.0
current_heading = 0.0
prev_heading_error = 0.0
last_gyro_time = START_TIME
next_gyro_time = START_TIME
steer_pwm = steer_mid


# Main loop 
while (time.time() - START_TIME) <= args.tim:
    current_time = time.time()
    elapsed_ms = round((current_time - START_TIME), 3)

    # Heading and steer correction loop 
    if current_time >= next_gyro_time:
        raw_gz = car.MPU_Read(6) 
        
        if raw_gz is not None:
            if abs(raw_gz) < 2.0:
                raw_gz = 0.0
                
            dt = current_time - last_gyro_time
            
            current_heading += (raw_gz * dt)
            
            heading_error = target_heading - current_heading
            delta_heading_error = (heading_error - prev_heading_error) / dt
            
            steer_correction = (args.Kp_steer * heading_error) + (args.Kd_steer * delta_heading_error)
            
            if steer_correction > 4.0:
                steer_correction = 4.0
            elif steer_correction < -4.0:
                steer_correction = -4.0
                
            steer_pwm = steer_mid + steer_correction
            car.set_steer_servo(steer_pwm)
            
            prev_heading_error = heading_error
            last_gyro_time = current_time
            
        next_gyro_time += 0.05 
    
    imgOG = get_safe_image(car)

    if imgOG is not None:
        cv2.imwrite('TrafficOG.jpg', imgOG)
    else:
        if imgOG is None:
            continue

    img = cv2.cvtColor(imgOG, cv2.COLOR_RGB2BGR)
    
    color = get_dominant_color(img, args.debug)

    raw_distance = car.read_distance()
    if raw_distance is None:
        distance = 999.0 
    else:
        distance = raw_distance

    duty = 0

    # Keep swivel fixed to the center 
    car.set_swivel_servo(swivel_mid)

    # Looks for the colors 
    if state == "Detecting":
        duty = 0 
        if color != "None":
            if distance < 200 and (color == "Yellow" or color == "Red"):
                state = "Object Close"
            else:
                state = "Driving"
                
    # If color is found, go to drive
    elif state == "Driving":
        if color == "None":  # Go back to detect if there is no color found 
            state = "Detecting" 
            continue

        # When the traffic light is near and not green --> Object is close
        if distance < 200 and (color == "Yellow" or color == "Red"):
            state = "Object Close"
            continue
            
        duty = 17 

    # If the color is green during the state "Object Close", continue driving normally 
    elif state == "Object Close":
        if color == "Green" or color == "None" or distance >= 200: 
            state = "Driving"
            continue

        # If the color is red, stop the car and switch state to Arrived 
        if color == "Red":
            if distance <= 170:
                duty = 0
                state = "Arrived"
            else:
                duty = 13 
        #Dynamic speed for yellow to slow down the car 
        elif color == "Yellow":
            if distance < 155:
                duty = 6
            elif distance < 200:
                duty = 8
            else:
                duty = 15

        # Emergency stop if the car is too close to the board, regardless of color 
        if distance <= 80: 
                duty = 0 
                state = "Arrived"

    # Stops at light 
    elif state == "Arrived":
        if color == "Green" or color == "Yellow": 
            state = "Driving" 
            continue
        duty = 0

    car.set_motor(duty)

    est_rps = duty / 8.35 
        
    print(f"{state:12} | Color: {color:6} | Dist: {distance:6.2f}cm | Duty: {duty:2}% | Est. RPS: {est_rps:5.2f} | Head: {current_heading:.1f}")

    time_arr.append(elapsed_ms)
    dist_arr.append(distance)
    duty_arr.append(duty)
    rps_arr.append(est_rps)
    
    time.sleep(args.delay)


car.stop()

# Write arrays to text file 
with open('TrafficLight.txt', 'w') as file:
    for j in range(len(time_arr)):
        file.write(f'{time_arr[j]:.3f}\t{dist_arr[j]:.2f}\t{duty_arr[j]}\t{rps_arr[j]:.3f}\n')
