import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from scipy.signal import savgol_filter

def wedge(phi):
    """ R^3 vector to so(3) matrix """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

def motion_model(R, omega, dt):
    R_pred = R @ expm(wedge(omega * dt))
    return R_pred

def measurement_Jacobain(g):
    H = wedge(g)
    return H

class right_iekf:
    def __init__(self):
        self.Phi = np.eye(3)               
        self.Q = 1e-4 * np.eye(3)          # Gyro noise
        self.N = 1e-4 * np.eye(3)          # Accel noise
        self.N_mag = 5e-3 * np.eye(3)      # Mag noise (higher so it doesn't fight Accel)
        self.f = motion_model              
        self.H = measurement_Jacobain      
        self.X = np.eye(3)
        self.P = 0.1 * np.eye(3)

    def prediction(self, omega, dt):
        self.X = self.f(self.X, omega, dt)
        F = expm(-wedge(omega * dt))
        self.P = self.P + F @ self.Q @ np.transpose(F)

    def correction(self, Y, g):
        # Accelerometer Correction (Pitch / Roll)
        Y_norm = Y / np.linalg.norm(Y)
        g_norm = g / np.linalg.norm(g)

        H = self.H(g_norm) 
        S = H @ self.P @ np.transpose(H) + self.N
        L = self.P @ np.transpose(H) @ np.linalg.inv(S)

        r = self.X @ Y_norm - g_norm 
        self.X = expm(wedge(L @ r)) @ self.X
        self.P = (self.Phi - L @ H) @ self.P

    def correction_mag(self, Y_mag, mag_ref):
        # Magnetometer Correction (Yaw)
        Y_norm = Y_mag / np.linalg.norm(Y_mag)
        ref_norm = mag_ref / np.linalg.norm(mag_ref)

        H = self.H(ref_norm) 
        S = H @ self.P @ np.transpose(H) + self.N_mag
        L = self.P @ np.transpose(H) @ np.linalg.inv(S)

        r = self.X @ Y_norm - ref_norm 
        self.X = expm(wedge(L @ r)) @ self.X
        self.P = (self.Phi - L @ H) @ self.P
        
print("Loading IMU data...")
df = pd.read_csv('data/imu_data_foot_raw.csv')

t = df['timestamp'].values / 1000000.0 # convert microseconds to seconds

# Extract sensor data (N x 3 arrays)
gyro = np.column_stack((df['gyro_x'], df['gyro_y'], df['gyro_z']))
accel = np.column_stack((df['accel_x'], df['accel_y'], df['accel_z']))

# Extract magnetometer data
mag = np.column_stack((df['mag_x'], df['mag_y'], df['mag_z']))

# Find your local magnetic North vector by averaging the first 50 stationary samples
mag_ref = np.mean(mag[:50], axis=0)

print("Initializing Filter...")
ekf = right_iekf()
gravity_vec = np.array([0, 0, 9.81])

# Initialize arrays to track our path
N = len(t)
positions = np.zeros((N, 3))
velocities = np.zeros((N, 3))

# ZUPT Thresholds
GYRO_STATIONARY_THRESH = 0.1   # rad/s
ACCEL_STATIONARY_MARGIN = 0.25 # m/s^2 deviation from resting magnitude
WARMUP_SAMPLES = 150    # At 50Hz, this is 3 seconds of stationary warmup
LOCAL_GRAVITY_MAG = 10.10

print("Processing Trajectory...")
for i in range(1, N):
    dt = t[i] - t[i-1]
    
    if dt <= 0:
        positions[i] = positions[i-1]
        velocities[i] = velocities[i-1]
        continue

    gyro_mag = np.linalg.norm(gyro[i])
    accel_mag = np.linalg.norm(accel[i])
    # ZUPT stuff TODO: play aorund with thresholds
    is_planted = (gyro_mag < GYRO_STATIONARY_THRESH) and (abs(accel_mag - LOCAL_GRAVITY_MAG) < ACCEL_STATIONARY_MARGIN)    
    
    ekf.prediction(gyro[i], dt)
    
    if is_planted or i < WARMUP_SAMPLES:
        ekf.correction(accel[i], gravity_vec)
        ekf.correction_mag(mag[i], mag_ref)
        
    accel_global = ekf.X @ accel[i]
    accel_linear = accel_global - np.array([0, 0, LOCAL_GRAVITY_MAG])
    
    if i < WARMUP_SAMPLES:
        velocities[i] = np.zeros(3)
        positions[i] = np.zeros(3)
    else:
        if is_planted:
            velocities[i] = np.zeros(3) 
        else:
            velocities[i] = velocities[i-1] + (accel_linear * dt)
            
        positions[i] = positions[i-1] + (velocities[i] * dt)
        
plt.figure(figsize=(10, 8))

plt.plot(positions[:, 0], positions[:, 1], label='Walking Path', color='purple', linewidth=2.5)
# # window_length = 51 (must be odd), polyorder = 3
# smoothed_x = savgol_filter(positions[:, 0], 51, 3)
# smoothed_y = savgol_filter(positions[:, 1], 51, 3)

# # Then plot the smoothed arrays instead of the raw ones:
# plt.plot(smoothed_x, smoothed_y, label='Smoothed Human Path', color='purple', linewidth=2.5)

plt.scatter(positions[0, 0], positions[0, 1], color='green', s=150, label='Start', marker='*')
plt.scatter(positions[-1, 0], positions[-1, 1], color='red', s=100, label='End')

plt.title('2D Pedestrian Trajectory (Foot-Mounted IMU)')
plt.xlabel('Global X Position (meters) [Positive = Left]')
plt.ylabel('Global Y Position (meters) [Positive = Forward]')

plt.axis('equal')    
plt.gca()#.invert_xaxis()  

plt.legend()
plt.grid(True)
plt.show()