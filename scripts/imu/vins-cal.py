import os.path, time
import numpy as np
import matplotlib.pyplot as plt

def clean_lines(file_name):
    with open(file_name, "r") as f:
        lines = f.readlines()

    # Remove the first 2 lines
    lines = lines[5:]

    # Remove the last 2 lines
    lines = lines[:-5]

    with open("cleaned_data.csv", "w") as f:
        f.writelines(lines)


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
  slope = np.polyfit(np.log10(dt), np.log10(arr), 1)[0]
  return slope / 2


def AllanDeviation(dataArr: np.ndarray, fs: float, maxNumM: int=20000):
    """Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html
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

# Config. params
RAW_DATA_FILE = 'vins-imu.csv'  # CSV data file "ax,ay,az,gx,gy,gz"
DATA_FILE = 'cleaned_data.csv'
fs = 1000  # Sample rate [Hz]

GYRO_FACTOR = 1.0
ACCEL_FACTOR = 1.0

print("last modified: %s" % time.ctime(os.path.getmtime(RAW_DATA_FILE)))
print("created: %s" % time.ctime(os.path.getctime(RAW_DATA_FILE)))

## Main
clean_lines(RAW_DATA_FILE)

# Load CSV into np array
dataArr = np.genfromtxt(DATA_FILE, delimiter=',')
ts = 1.0 / fs

# Separate into arrays
ax = dataArr[:, 1]
ay = dataArr[:, 2]
az = dataArr[:, 3]
gx = dataArr[:, 4]
gy = dataArr[:, 5]
gz = dataArr[:, 6]

# Calculate gyro angles
thetax = np.cumsum(gx) * ts  
thetay = np.cumsum(gy) * ts
thetaz = np.cumsum(gz) * ts

# Compute Allan deviations
Nlen = 40000
(taux, gdx) = AllanDeviation(thetax, fs, maxNumM=Nlen)
(tauy, gdy) = AllanDeviation(thetay, fs, maxNumM=Nlen)
(tauz, gdz) = AllanDeviation(thetaz, fs, maxNumM=Nlen)

rwx = calc_slope(taux, gdx)
rwy = calc_slope(tauy, gdy)
rwz = calc_slope(tauz, gdz)

x_len = len(tauz)
y_len = len(gdz)
ref_x = [1e-3, 1.997]
ref_y = [1e-1, 1e-3]

wx = calculate_white_noise(gx)
wy = calculate_white_noise(gy)
wz = calculate_white_noise(gz)
white_noise = np.sqrt(wx*wx + wy*wy + wz*wz)
print("Gyro White Noise: ", GYRO_FACTOR*white_noise)

random_walk = np.sqrt(rwx*rwx + rwy*rwy + rwz*rwz)
print("Gyro Random Walk ", random_walk)


#############

# Calculate gyro angles
thetaxa = np.cumsum(ax) * ts  
thetaya = np.cumsum(ay) * ts
thetaza = np.cumsum(az) * ts

# Compute Allan deviations
(tauxa, adx) = AllanDeviation(thetaxa, fs, maxNumM=Nlen)
(tauya, ady) = AllanDeviation(thetaya, fs, maxNumM=Nlen)
(tauza, adz) = AllanDeviation(thetaza, fs, maxNumM=Nlen)

awx = calc_slope(tauxa, adx)
awy = calc_slope(tauya, ady)
awz = calc_slope(tauza, adz)

wxa = calculate_white_noise(ax)
wya = calculate_white_noise(ay)
wza = calculate_white_noise(az)
accel_white_noise = np.sqrt(wxa*wxa + wya*wya + wza*wza)
print("Accel White Noise: ", ACCEL_FACTOR*accel_white_noise)

accel_random_walk = np.sqrt(awx*awx + awy*awy + awz*awz)
print("Accel Random Walk ", accel_random_walk)

#############

# Plot data on log-scale
plt.figure()
plt.title('Gyro Allan Deviations')
plt.plot(taux, gdx, label='gx')
plt.plot(tauy, gdy, label='gy')
plt.plot(tauz, gdz, label='gz')
plt.plot(ref_x, ref_y, '--', label='ref (m=-0.5)')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Deviation [deg/sec]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()

# Plot data on log-scale
plt.figure()
plt.title('Accel Allan Deviations')
plt.plot(tauxa, adx, label='ax')
plt.plot(tauya, ady, label='ay')
plt.plot(tauza, adz, label='az')
plt.plot(ref_x, ref_y, '--', label='ref (m=-0.5)')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Deviation [deg/sec]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()

