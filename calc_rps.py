import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math

def movingAvg(arr, position, numvals=3, wrap=1):
    sumvals    = 0
    count      = 0  
    array_size = len(arr)
    
    for i in range(numvals):
        if (position - i >= 0 and position - i < array_size):
            sumvals = sumvals + arr[(position - i)]
            count   = count + 1
        elif (position - i < 0 and wrap == 1): 
            sumvals = sumvals + arr[(position - i)]
            count   = count + 1
        elif (position - i > array_size and wrap == 1):
            sumvals = sumvals + arr[(position - i)%array_size]
            count   = count + 1
            
    return sumvals/count

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--file', action='store', type=str, required=True, help='Choose which file to plot')
args = parser.parse_args()

fname = args.file
file = open(fname, "r")
data = file.read().splitlines()
MAXSIZE = len(data) - 1
DEBUG = False

time = [0] * MAXSIZE
value = [0] * MAXSIZE
delta = [0] * MAXSIZE
avg = [0] * MAXSIZE
i = 0

for dat in data[1:]:
    values = dat.split()
    if len(values) < 2:
        continue
    time[i] = float(values[0])
    value[i] = float(values[1])
    if i == 0:
        delta[i] = 0.0
    else:
        delta[i] = value[i] - value[i-1]
    i = i + 1

i = 0

for times in time:
    avg[i] = movingAvg(delta, i)
    i = i + 1

max_val = max(avg)
min_val = min(avg)

pos_thresh = max_val * 0.20
neg_thresh = min_val * 0.20
thresh = pos_thresh + neg_thresh

values_arr = [0] * MAXSIZE
high = True
transitions = 0
i = 0

for val in avg:
    if high and val > pos_thresh:
        transitions += 1
        high = False
    elif not high and val < neg_thresh:
        transitions += 1
        high = True
        
    if val > thresh:
        values_arr[i] = 1
    elif val < thresh:
        values_arr[i] = -1
    i = i + 1

total_time = time[len(time) - 1]

if total_time > 0:
    RPS = (transitions / total_time) * (1/4)
else:
    RPS = 0.0
    
print(f'Total transitions: {transitions} \t RPS: {RPS:.2f} \n')

plt.figure(figsize=(10, 10))

plt.subplot(3, 1, 1)
plt.plot(time, value, 'b-', label="Raw A/D Output")
plt.title('A/D Output')
plt.ylabel('Sensor Value')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, delta, 'r-', label="Change in Output")
plt.title('Delta of Output')
plt.ylabel('Delta')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, values_arr, 'b', label="Detected State")
plt.title('Binary Transition State')
plt.xlabel('Time (sec)')
plt.ylabel('State')
plt.grid(True)

output_filename = fname.replace('.txt', '_plot.png')
plt.tight_layout() 
plt.savefig(output_filename)
print(f"Plot saved successfully as: {output_filename}")
