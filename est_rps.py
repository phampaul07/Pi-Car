import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.fftpack import fft

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

N = 512
T = (time[-1] - time[0]) / (len(time) - 1)
sliced_delta = delta[-N:]
t = np.linspace(0, N*T, N)

delta_fft = fft(sliced_delta)
y_data = 2/N * np.abs(delta_fft[0:N//2])
freq = np.linspace(0, 1/(2*T), N//2)

valid_indices = np.where((freq > 1.0) & (freq < 20.0))[0]
best_valid_index = np.argmax(y_data[valid_indices])
peak_index = valid_indices[best_valid_index]

peak_freq = freq[peak_index]
RPS = (peak_freq / 2.0)

print(f"Dominant Frequency: {peak_freq:.2f} Hz")
print(f"Estimated RPS: {RPS:.2f} RPS")

plt.plot(freq, y_data)
plt.grid(True)
plt.xlabel("freq - Hz")

plt.tight_layout()
plot_name = fname.replace('.txt', '_fft.png')
plt.savefig(plot_name)
print(f"Saved plot as {plot_name}")
