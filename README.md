# Template_Module_10

This module is for the final project.

## PiCar Configuration

Previously, there were CONFIG.txt files for each of the cars for setting the servo-motor ranges.The hardware
is swapped too frequently to maintain these configuration files for each of the cars. Please run the 
configure_servos.py script in the repository to create your own configuration file for that particular PiCar.

## It includes
- Several example data files, good and one bad
- configure_servos.py - This program is used for configuring each of the servo-motors ranges and "center".
  (Note: low-end means left (swivel, steer), down (nod) and high-end means right (swivel, steer), up (nod))
- check_servos.py - This program is good for checking the span of the servos for a given configuration file
- script1.py (and 2,3,4) - These programs are from Module 9 and can be used on the real PiCar for this module

## Deliverables
- objective1.py # Control on noload car
- objective2.py # Movement with control
- objective3.py # Seeker
- objective4.py # Traffic light
- objective5.py # Group's choice
- car_noload_4rps.txt (Objective #1)
- manual_car_[specified-speed]rps.txt (Objective #2)
- Seeker.txt (Objective #3)
- Traffic_light.txt (Objective #4)
- Objective_5.txt(or png) (Objective #5)


## NOTE:
For data files, data_example.txt and data_example2.txt are examples of acceptable data files.  
They include the sampling time and note that the second file includes more data than what was 
being asked for, which is ok, as long as the required elements are present.

data_example_bad.txt has two problems.  
- it is missing the sampling period to start the first line
- note that the time samples are not equally spaced

Also, do not include extra lines without data.  So if you have 10,000 element arrays for example,
but only 2000 lines that contain actual values, do not inculde 8000 lines of 0's.
