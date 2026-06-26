import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--file', action='store', type=str, required=True)
args = parser.parse_args()

fname = args.file
file = open(fname, "r")
data = file.read().splitlines()

time = []
RPS = []

for dat in data[1:]:
    values = dat.split()
    if len(values) >= 3:
        time.append(float(values[0]))
        RPS.append(float(values[2]))

for k in range(1, len(RPS)):
    if RPS[k] == 0 and RPS[k-1] > 0:
        RPS[k] = RPS[k-1]

max_time = max(time)

plt.figure(figsize=(10, 6))

plt.xlim(0, max_time)

xmarks = np.linspace(0, max_time, 10)
plt.xticks(xmarks)

plt.plot(time, RPS, 'b', label="Velocity (RPS)")

plt.grid(True)
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('RPS')
plt.title(f'Velocity Plot: {fname}')

plot_name = fname.replace('.txt', '.png')
plt.savefig(plot_name)
print(f'Plot Saved as {plot_name}')