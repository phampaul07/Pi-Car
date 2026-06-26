from picar import PiCar, test, configure
import time
import argparse

# Argparsing and optimal PID control values for loaded ground drive
parser = argparse.ArgumentParser(description='PID Control for PiCar')
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
parser.add_argument('--tim', action='store', type=float, default=10.0)
parser.add_argument('--adSample', action='store', type=float, default=0.005)
parser.add_argument('--adDelay', action='store', type=float, default=0)
parser.add_argument('--speedCalc', action='store', type=float, default=0.1)
parser.add_argument('--motorDelay', action='store', type=float, default=1.0)
parser.add_argument("--debug", action="store_true")
parser.add_argument ('--duty', action='store', type=float, default= None, help= 'Duty Cyle')
parser.add_argument('--rps', action='store', type=float, default=4.0)
parser.add_argument('--Kp', action='store', type=float, default=11.0)
parser.add_argument('--Ki', action='store', type=float, default=7.0)
parser.add_argument('--Kd', action='store', type=float, default=0.1)
args = parser.parse_args()

# Car and array initialization 
car = PiCar(mock_car=args.mock_car, threaded= True)
MAXSIZE = 10000
AD_reading = [0] * MAXSIZE
time_arr = [0] * MAXSIZE
RPS = [0] * MAXSIZE
total_trans_arr = [0] * MAXSIZE
total_trans = 0
i = 0

# Use ~~ 36% of the open loop gain 
rps_to_duty = 8.35
duty = args.rps * rps_to_duty
if args.duty is not None:
    duty = args.duty
if duty > 100: duty = 100

START_TIME = time.time()
next_ad_time = START_TIME + args.adDelay
next_calc_time = START_TIME + args.adDelay
next_sensor_time = START_TIME
ultra_sensor_rate = 0.1 #Reads distance 10 times a second 
# System states
motor_on = False
wall_detected = False
distance = 0 

# PID tracking variables 
error = 0.0
prev_error = 0.0
SE = 0.0
new_duty = duty
current_est_rps = 0.0

# Initialize steer 
steer_pwm = 0
car.set_nod_servo(steer_pwm)
step = 0.8

while (time.time() - START_TIME) <= args.tim:
    current_time = time.time()
    total_time = current_time - START_TIME
    
    if not motor_on and total_time >= args.motorDelay and wall_detected == False:
        car.set_motor(duty)
        motor_on = True

    # AD Readings
    if current_time >= next_ad_time:
        bit = car.adc.read_adc(0)
        elapsed = round((current_time - START_TIME), 3)
    
        if i > 0:
            window = AD_reading[max(0, i-100):i]
            midline = min(window) + (max(window) - min(window)) * 0.5
            if AD_reading[i-1] <= midline and bit > midline:
                total_trans += 1

        AD_reading[i] = bit
        time_arr[i] = elapsed
        total_trans_arr[i] = total_trans
        RPS[i] = current_est_rps
    
        i += 1
        next_ad_time += args.adSample

    # Ultrasonic sensor and wall detection 
    if current_time >= next_sensor_time:
        distance = car.read_distance()
        if distance is not None and distance <= 100:
            if not wall_detected: 
                print("Distance Reached: Motors Stopped")
            car.set_motor(0)
            motor_on = False
            current_est_rps = 0.0 
            wall_detected = True  # Stops the PID loop from turning the motors after wall is detected. 
        next_sensor_time += ultra_sensor_rate

    # RPS calculation and PID controls 
    if current_time >= next_calc_time:
        if i >= 300 and motor_on:
            window_calc = AD_reading[i - 300: i]
            MAX = max(window_calc)
            MIN = min(window_calc)
            deltaThresh = MAX - MIN
    
            write_idx = i - 1
            est_rps = 0.0

            if deltaThresh > 50:
                thresh = MIN + (deltaThresh * 0.5)
                transitions_found = 0
                firstTrans = 0.0
                fifthTrans = 0.0
                idx = write_idx
    
                while idx > (i - 300) and transitions_found < 5:
                    if (AD_reading[idx] > thresh and AD_reading[idx-1] <= thresh) or \
                       (AD_reading[idx] <= thresh and AD_reading[idx-1] > thresh):
                        transitions_found += 1
                        if transitions_found == 1:
                            firstTrans = time_arr[idx]
                        if transitions_found == 5:
                            fifthTrans = time_arr[idx]
                    idx -= 1

                if transitions_found == 5:
                    delta_t = firstTrans - fifthTrans
                    if delta_t > 0:
                        est_rps = 1.0 / delta_t
    
            current_est_rps = est_rps
            error = args.rps - est_rps
    
            if motor_on:
                SE += error * args.speedCalc
                if SE > 20.0: SE = 20.0
                elif SE < -20.0: SE = -20.0

            delta_error = (error - prev_error) / args.speedCalc
            prev_error = error

            correction = (args.Kp * error) + (args.Ki * SE) + (args.Kd * delta_error)
            new_duty = duty + correction
    
            if new_duty > 100.0: new_duty = 100.0
            elif new_duty < 0.0: new_duty = 0.0
    
            car.set_motor(new_duty)

            if args.debug:
                dist_str = f"{distance:.2f}" if distance is not None else "N/A"
                print(f"Time: {time_arr[write_idx]:.2f}s | RPS: {est_rps:.2f} | Duty: {new_duty:.2f} | Distance: {dist_str}")

        # Manual control over the PiCar's steer 
        key = car.get_keyin()
        if key == 'd':
            steer_pwm -= step
            if steer_pwm <= -10:
                steer_pwm = -10
            car.set_steer_servo(steer_pwm)

        elif key == 'a':
            steer_pwm += step
            if steer_pwm >= 10:
                steer_pwm = 10
            car.set_steer_servo(steer_pwm)
        next_calc_time += args.speedCalc

car.stop()

# Save the arrays into a text file 
with open(f'manual_car_{args.rps}rps.txt', 'w') as file:
    file.write(f'{args.adSample:.4f}\n')
    for j in range(i):
        file.write(f'{time_arr[j]:.4f}\t {AD_reading[j]:.0f}\t {RPS[j]:.3f} \n')
