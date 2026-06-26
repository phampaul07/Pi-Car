from picar import PiCar
import time
import argparse

# Threshold to turn 
TRIGGER_DIST_CM = 80.0

STEER_CENTER = 0.0

# Parameters for the perform_3_point_turn function 
REVERSE_PWM = 17
TURN_PWM_FWD = 17
TURN_PWM_REV = 17

STEER_MAX_LEFT = 10
STEER_MAX_RIGHT = -10

TIME_INITIAL_REVERSE = 2.0
TIME_TURN_1_FWD = 2.0   
TIME_TURN_2_REV = 2.0   
TIME_TURN_3_FWD = 1.7 


# Performs a 3-point turn in order to make the car turn 180 degrees 
def perform_3_point_turn(car):
    print("\n Executing 3-Point Turn...")
    
    car.set_motor(0)
    time.sleep(0.5)

    print("Reverse Straight")
    car.set_steer_servo(STEER_CENTER)
    car.set_motor(REVERSE_PWM, forward=False)
    time.sleep(TIME_INITIAL_REVERSE)
    car.set_motor(0)
    time.sleep(1)

    print("Max Left, Forward")
    car.set_steer_servo(STEER_MAX_LEFT)
    time.sleep(1) 
    car.set_motor(TURN_PWM_FWD)
    time.sleep(TIME_TURN_1_FWD)
    car.set_motor(0)
    time.sleep(1)

    print("Max Right, Reverse")
    car.set_steer_servo(STEER_MAX_RIGHT)
    time.sleep(1) 
    car.set_motor(TURN_PWM_REV, forward=False)
    time.sleep(TIME_TURN_2_REV)
    car.set_motor(0)
    
    time.sleep(1.0)

    print("Max Left, Forward")
    car.set_steer_servo(STEER_MAX_LEFT)
    time.sleep(0.5)
    car.set_motor(TURN_PWM_FWD)
    time.sleep(TIME_TURN_3_FWD)
    car.set_motor(0)

    print("Turn Complete. Centering.")
    car.set_steer_servo(STEER_CENTER)
    time.sleep(1.0)
    print("Resuming Cruise Control...\n")

# Argument Parsers 
parser = argparse.ArgumentParser(description='Objective 5: IMU Heading Hold')
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--tim', action='store', type=float, default=60.0)
parser.add_argument('--adSample', action='store', type=float, default=0.005)
parser.add_argument('--speedCalc', action='store', type=float, default=0.1)
parser.add_argument('--motorDelay', action='store', type=float, default=1.0)
parser.add_argument("--debug", action="store_true")
parser.add_argument('--rps', action='store', type=float, default=4.0)

# Speed PID variables
parser.add_argument('--Kp', action='store', type=float, default=11.0)
parser.add_argument('--Ki', action='store', type=float, default=7.0)
parser.add_argument('--Kd', action='store', type=float, default=0.1)

# Steering PID variables
parser.add_argument('--Kp_steer', action='store', type=float, default=0.2)
parser.add_argument('--Kd_steer', action='store', type=float, default=0.0)
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)

# Fix car servos to the center 
car.set_nod_servo(0) 
car.set_swivel_servo(0) 
car.set_steer_servo(STEER_CENTER)

# Initialize arrays 
MAXSIZE = 10000
AD_reading = [0] * MAXSIZE
time_arr = [0] * MAXSIZE
RPS = [0] * MAXSIZE
total_trans = 0
i = 0

rps_to_duty = 8.35
duty = args.rps * rps_to_duty
if duty > 100: duty = 100

START_TIME = time.time()
next_ad_time = START_TIME
next_calc_time = START_TIME
next_dist_time = START_TIME
# States to determine whether to run the PID control loops and the reverse functions 
motor_on = False
has_reversed = False 

# Speed PID states
error = 0.0
prev_error = 0.0
SE = 0.0
new_duty = duty
current_est_rps = 0.0
raw_distance = None

# Accelerometer parameters 
target_heading = 0.0
current_heading = 0.0
prev_heading_error = 0.0
last_gyro_time = START_TIME
next_gyro_time = START_TIME
steer_pwm = STEER_CENTER

while (time.time() - START_TIME) <= args.tim:
    current_time = time.time()

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
                
            steer_pwm = STEER_CENTER + steer_correction
            car.set_steer_servo(steer_pwm)
            
            prev_heading_error = heading_error
            last_gyro_time = current_time
            
        next_gyro_time += 0.05 

    # Ultrasonic and motor control 
    if not motor_on and (current_time - START_TIME) >= args.motorDelay:
        car.set_motor(duty)
        motor_on = True
        
    if current_time >= next_dist_time:
        raw_distance = car.read_distance()
        next_dist_time += 0.1 
    
    if raw_distance is not None:
        if raw_distance <= TRIGGER_DIST_CM:
            
            # Hits the wall for the first time 
            if not has_reversed:
                car.set_motor(0)
                perform_3_point_turn(car)
                
                # Flag the turn
                has_reversed = True
                
                # Reset speed PID
                SE = 0.0
                prev_error = 0.0
                current_est_rps = 0.0
                new_duty = duty
                
                # Reset heading states
                current_heading = 0.0
                prev_heading_error = 0.0
                steer_pwm = STEER_CENTER
                car.set_steer_servo(steer_pwm)
                
                current_time = time.time()
                next_ad_time = current_time
                next_calc_time = current_time
                next_dist_time = current_time
                next_gyro_time = current_time
                last_gyro_time = current_time
                continue 
                
            # Second time hitting the wall --> stop motor 
            else:
                print("\n Second wall reached! Stopping operation.")
                car.set_motor(0)
                motor_on = False
                break 

    # PID Controls 
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
        RPS[i] = current_est_rps
        i += 1
        next_ad_time += args.adSample

    if current_time >= next_calc_time:
        if i >= 300 and motor_on:
            window_calc = AD_reading[i - 300: i]
            MAX = max(window_calc)
            MIN = min(window_calc)
            deltaThresh = MAX - MIN
    
            est_rps = 0.0
            if deltaThresh > 50:
                thresh = MIN + (deltaThresh * 0.5)
                transitions_found = 0
                firstTrans = 0.0
                fifthTrans = 0.0
                idx = i - 1
    
                while idx > (i - 300) and transitions_found < 5:
                    if (AD_reading[idx] > thresh and AD_reading[idx-1] <= thresh) or \
                       (AD_reading[idx] <= thresh and AD_reading[idx-1] > thresh):
                        transitions_found += 1
                        if transitions_found == 1: firstTrans = time_arr[idx]
                        if transitions_found == 5: fifthTrans = time_arr[idx]
                    idx -= 1

                if transitions_found == 5:
                    delta_t = firstTrans - fifthTrans
                    if delta_t > 0:
                        est_rps = 1.0 / delta_t
    
            current_est_rps = est_rps
            speed_error = args.rps - est_rps
    
            SE += speed_error * args.speedCalc
            if SE > 20.0: SE = 20.0
            elif SE < -20.0: SE = -20.0

            delta_speed_error = (speed_error - prev_error) / args.speedCalc
            prev_error = speed_error

            correction = (args.Kp * speed_error) + (args.Ki * SE) + (args.Kd * delta_speed_error)
            new_duty = max(0.0, min(100.0, duty + correction))
    
        if motor_on:
            # Downhill active braking
            if current_est_rps > (args.rps + 1.5):
                car.set_motor(20, forward=False)
                print ("Too Fast! Slowing Down.")
                SE = 0.0 
            else:
                car.set_motor(new_duty)

        if args.debug:
            print(f"Time: {elapsed:.2f}s | RPS: {current_est_rps:.2f} | Head: {current_heading:.1f}deg | Steer: {steer_pwm:.1f}")
        
        next_calc_time += args.speedCalc

car.stop()

# Write arrays to text 
with open('Objective_5.txt', 'w') as file:
    for j in range(i):
        file.write(f'{time_arr[j]:.4f}\t{AD_reading[j]:.0f}\t{RPS[j]:.3f}\n')
