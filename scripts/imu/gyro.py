import numpy as np
import matplotlib.pyplot as plt

def calculate_white_noise(data):
#  mean = np.mean(data)
#  variance = np.var(data)
  std_dev = np.std(data)
  return std_dev

def calc_slope(x,y):
   slope = np.polyfit(x, y, 1)[0]
   #gyroscope_random_walk = np.power(2, slope - 1) * 1e-6
   gyroscope_random_walk = slope * 2
   return gyroscope_random_walk


def calculate_random_walk(arr, dt):
#  slope = np.polyfit(np.log10(np.arange(1, len(arr) + 1)), np.log10(arr), 1)[0]
  slope = np.polyfit(np.log10(dt), np.log10(arr), 1)[0]
  return slope / 2

# Config. params
DATA_FILE = 'gyro.csv'  # CSV data file "gx,gy,gz"
#DATA_FILE = 'accel.csv'  # CSV data file "gx,gy,gz"
fs = 1020  # Sample rate [Hz]

def AllanDeviation(dataArr: np.ndarray, fs: float, maxNumM: int=10000):
    """Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

    Args
    ----
        dataArr: 1D data array
        fs: Data sample frequency in Hz
        maxNumM: Number of output points

    Returns
    -------
        (taus, allanDev): Tuple of results
        taus (numpy.ndarray): Array of tau values
        allanDev (numpy.ndarray): Array of computed Allan deviations
    """
    ts = 1.0 / fs
    N = len(dataArr)
    Mmax = 2**np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)  # Round up to integer
    M = np.unique(M)  # Remove duplicates
    taus = M * ts  # Compute 'cluster durations' tau

    # Compute Allan variance
    allanVar = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allanVar[i] = np.sum(
            (dataArr[twoMi:N] - (2.0 * dataArr[mi:N-mi]) + dataArr[0:N-twoMi])**2
        )

    allanVar /= (2.0 * taus**2) * (N - (2.0 * M))
    return (taus, np.sqrt(allanVar))  # Return deviation (dev = sqrt(var))


# Load CSV into np array
dataArr = np.genfromtxt(DATA_FILE, delimiter=',')
ts = 1.0 / fs

# Separate into arrays
gx = dataArr[:, 0]
gy = dataArr[:, 1]
gz = dataArr[:, 2]

# Calculate gyro angles
thetax = np.cumsum(gx) * ts  
thetay = np.cumsum(gy) * ts
thetaz = np.cumsum(gz) * ts

# Compute Allan deviations
Nlen = 5000
(taux, adx) = AllanDeviation(thetax, fs, maxNumM=Nlen)
(tauy, ady) = AllanDeviation(thetay, fs, maxNumM=Nlen)
(tauz, adz) = AllanDeviation(thetaz, fs, maxNumM=Nlen)

rwx = calc_slope(taux, adx)
rwy = calc_slope(tauy, ady)
rwz = calc_slope(tauz, adz)

random_walk = np.sqrt(rwx*rwx + rwy*rwy + rwz*rwz)
print("Random Walk ", random_walk)


x_len = len(tauz)
y_len = len(adz)
ref_x = [1e-3, 1.997]
ref_y = [1e-1, 1e-3]
#ref_x = [tauz[0], 1]
#ref_y_f = adz[0] + (-0.5 * (tauz[x_len-1] - tauz[0]))
#ref_y_f = adz[0] + (-0.5 * (tauz[x_len-1] - tauz[0]))
#ref_y = [adz[0], 1e-3]

wx = calculate_white_noise(gx)
wy = calculate_white_noise(gy)
wz = calculate_white_noise(gz)
white_noise = np.sqrt(wx*wx + wy*wy + wz*wz)
print("white noise: ", white_noise)

# Plot data on log-scale
plt.figure()
plt.title('Gyro Allan Deviations')
plt.plot(taux, adx, label='gx')
plt.plot(tauy, ady, label='gy')
plt.plot(tauz, adz, label='gz')
plt.plot(ref_x, ref_y, '--', label='ref (m=-0.5)')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Deviation [deg/sec]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()

