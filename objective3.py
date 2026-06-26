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
# Processes the image to find and mask the blue object, calculates the center of mass, and returns the angle from the center of the camera frame. 
def get_blue_angle(img, debug=False):
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (115, 150, 50), (125, 255, 255))
    tape_image = img.copy()

    M = cv2.moments(mask)
    if M["m00"] == 0:
        if debug:
            print("No blue objects present")
        return 360.0

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    theta = math.atan2(cX - w/2, h - cY)
    angle_degrees = math.degrees(theta)
    if debug:
        COM = cv2.circle(tape_image, (cX, cY), 5, (0, 0, 255), 2)
        print(f"Center: ({cX} , {cY})")
        print(f'Angle from camera: {angle_degrees:.2f} degrees')
        cv2.imwrite('Original.jpg', img)
        cv2.imwrite('Mask.jpg', mask)
        cv2.imwrite('Center_of_Mass.jpg', COM)
    return angle_degrees

# Argparse 
parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--tim', action='store', type=int, default=10, help='Time for program to run in seconds')
parser.add_argument('--delay', action='store', type=float, default=0.05, help='Delay between readings')
parser.add_argument("--debug", action="store_true", help="Print debug info to console")
parser.add_argument('--delta', action='store', type=float, default=0.5, help='Change in Shift')
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
args = parser.parse_args()

START_TIME = time.time()

# Initializes state for detecting 
ratio = 0.0472
state = "Detecting"
step = 0.5
pwm = 0

current_steer = 0
search_dir = 1

# Data arrays 
time_arr = []
dist_arr = []
duty_arr = []

# Hardware limits 
swivel_low = -10
swivel_high = 10
swivel_mid = 0

steer_low = -10
steer_high = 10
steer_mid = 0

# Initialize PiCar
car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_swivel_servo(swivel_mid)
car.set_steer_servo(steer_mid)
car.set_nod_servo(0.5)
time.sleep(1)

# Main loop 
while (time.time() - START_TIME) <= args.tim:
    elapsed_ms = round((time.time() - START_TIME), 3)
    
    imgOG = get_safe_image(car)
        
    if imgOG is None:  # If the camera lags and can not take a picture, continue to run the program 
         continue
         
    img = cv2.cvtColor(imgOG, cv2.COLOR_RGB2BGR)
    angle = get_blue_angle(img, args.debug)

    
    distance = car.read_distance()
    
    if distance is None:
        distance = 999.0

    duty = 0

    # State "Detecting" - Sweeps the room to find blue object. 
    if state == "Detecting":
        print (f"{state}... Distance: {distance:.2f}cm ")
        car.set_steer_servo(steer_mid)
        car.set_motor(0)

        # Sweep the swivel servo left to right, back and forth. 
        pwm += step
        if pwm >= swivel_high:
            step = -0.5
        elif pwm <= swivel_low:
            step = 0.5
        car.set_swivel_servo(pwm)

        # If it finds an angle, there means there is a blue object, and switches to the Align state. 
        if angle != 360.0:
            search_dir = 1 if pwm > 0 else -1
            car.set_swivel_servo(swivel_mid)
            state = "Aligning"

    # Aligning state attempts to align the center of mass of the blue object with the center of the camera
    elif state == "Aligning":
        print (f"{state}... Distance: {distance:.2f}cm ")
        car.set_swivel_servo(swivel_mid)

        # If it loses the angle, it loses the blue object. Return to detect 
        if angle == 360.0:
            car.set_motor(0)
            state = "Detecting"
            continue

        # If the target is centered (or about 17 degrees or less) and far, switch to the drive fast state. 
        if abs(angle) < 17 and distance >= 300:
            state = "Driving"
            continue

        # Steering controller, with the max steers being -3 to 3. 
        current_steer = -angle * 0.35
        if current_steer > 3:
            current_steer = 3
        elif current_steer < -3:
            current_steer = -3
            
        car.set_steer_servo(current_steer)

        # Dynamic speed control based on distance 
        if distance <= 40:
            duty = 0
            print (f"Near Object. Stop motor. Distance: {distance:.2f}cm ")
            state = "Arrived"
        elif distance < 85:
            duty = 8
        elif distance < 150:
            duty = 10
        elif distance < 260: 
            duty = 17
        elif distance < 300:
            duty = 25
        else:
            duty = 40
            
        car.set_motor(duty)

    elif state == "Driving":
        print (f"{state}... Distance: {distance:.2f}cm ")
        car.set_swivel_servo(swivel_mid)
        current_steer = steer_mid
        car.set_steer_servo(current_steer)

        # Go back to aligning if the car drifts off course or gets too close
        if angle == 360.0 or abs(angle) > 30 or distance < 300:
            state = "Aligning"
            continue

        # Dynamic speed control 
        if distance <= 40: 
            duty = 0 
            print (f"Near Object. Stop motor. Distance: {distance: .f}cm ") 
            state = "Arrived" 
        elif distance < 85: 
            duty = 8
        elif distance < 150:
            duty = 10
        elif distance < 260:  
            duty = 17
        elif distance < 300: 
            duty = 25
        elif distance < 350:
            duty = 35
        elif distance < 400:
            duty = 40
        elif distance < 450:
            duty = 45
        else:
            duty = 50
        
        car.set_motor(duty)

    # State turns to arrive. Target has been secured and stop motors. 
    elif state == "Arrived":
        print (f"{state}! Target Secured. Distance: {distance:.2f}cm ")
        duty = 0
        car.set_motor(0)
        car.set_swivel_servo(swivel_mid)
        car.set_steer_servo(steer_mid)

    time_arr.append(elapsed_ms)
    dist_arr.append(distance)
    duty_arr.append(duty)

    if args.debug:
        print(f'Time: {elapsed_ms}s | Distance: {distance:.2f}cm | Swivel: {pwm:.1f} | Steer: {current_steer:.1f} | Duty: {duty}%')
        
    time.sleep(args.delay)
car.stop()

# Save the arrays to Seeker.txt file
with open('Seeker.txt', 'w') as file:
    file.write(f'Time(s)\tDistance(cm)\tDuty(%)\n')
    for j in range(len(time_arr)):
        file.write(f'{time_arr[j]:.3f}\t{dist_arr[j]:.2f}\t{duty_arr[j]}\n')
