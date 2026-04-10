import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R
import os

def wedge(phi):
    """
    R^3 vector to so(3) matrix
    @param  phi: R^3
    @return Phi: so(3) matrix
    """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

def adjoint(R_mat):
    """
    Adjoint of SO3 Adjoint (R) = R
    """
    return R_mat

#############################################################################
#                    TODO: Implement your code here                         #
#############################################################################
def motion_model(R_mat, omega, dt):
    """
    @param  R:      State variable (3, 3)
    @param  omega:  gyroscope reading (3,)
    @param  dt:     time step
    @return R_pred: predicted state variable (3, 3)
    """
    omega_hat = wedge(omega * dt)
    # FIX: Post-multiply to apply rotation in the Body frame, not World frame
    R_pred = R_mat @ expm(omega_hat) 
    return R_pred

def measurement_Jacobain(g):
    """
    @param  g: gravity (3,)
    @return H: measurement Jacobain (3, 3)
    """
    H = wedge(g)
    return H

#############################################################################
#                            END OF YOUR CODE                               #
#############################################################################

class right_iekf:

    def __init__(self):
        self.Phi = np.eye(3)               # state transtion matrix
        self.Q = 1e-4*np.eye(3)            # gyroscope noise covariance
        # FIX: Increased N slightly so walking acceleration isn't treated as purely gravity
        self.N = 10*np.eye(3)            # accelerometer noise covariance
        self.f = motion_model              # process model
        self.H = measurement_Jacobain      # measurement Jacobain
        self.X = np.eye(3)                 # state vector
        self.P = 0.1 * np.eye(3)           # state covariance

    def prediction(self, omega, dt):
        """
        @param omega: gyroscope reading
        @param dt:    time step
        """
        self.X = self.f(self.X, omega, dt)
        self.P = self.P + self.Q
        return

    def correction(self, Y, g):
        """
        @param Y: linear acceleration measurement
        @param g: gravity
        """
        H = self.H(g)
        N = self.N
        S = H @ self.P @ H.T + N 

        # Kalman gain
        L = self.P @ np.transpose(H) @ np.linalg.inv(S)

        # Innovation (measurement residual)
        r = self.X @ Y - g

        # Update state using exponential map (SO(3) correction)
        self.X = expm(wedge(L @ r)) @ self.X

        # Update covariance
        self.P = (np.eye(3) - L @ H) @ self.P @ (np.eye(3) - L @ H).T + L @ self.N @ L.T
        return

def riekf_load_data(file_path):
    df = pd.read_csv(file_path)
    data = {}
    
    data['accel'] = df[['accel_x', 'accel_y', 'accel_z']].values
    data['omega'] = df[['gyro_x', 'gyro_y', 'gyro_z']].values
    
    timestamps = df['timestamp'].values
    dt = np.zeros(len(timestamps))
    dt[1:] = (timestamps[1:] - timestamps[:-1]) / 1e6
    dt[0] = dt[1] if len(dt) > 1 else 0.01 
    data['dt'] = dt
    
    data['gravity'] = np.array([0.0, 0.0, 9.81])
    data['euler_gt'] = None 
    
    return data

def ahrs_riekf(iekf_filter, data):
    accel = data['accel']
    omega = data['omega']
    dt = data['dt']
    gravity = data['gravity']
    N = data['accel'].shape[0]

    states_rot = np.zeros((N+1, 3, 3))
    states_rot[0] = iekf_filter.X
    
    # --- Variables for Dead Reckoning ---
    positions = np.zeros((N+1, 3))
    v = np.zeros(3) # Velocity
    p = np.zeros(3) # Position
    
    # --- ZUPT Thresholds ---
    # Adjust these based on the noise profile of your specific sensor
    ACCEL_THRESHOLD = 1.0  # Allowed deviation from 9.81 m/s^2
    GYRO_THRESHOLD = 0.4   # Allowed rotation rate in rad/s when "still"
    
    for i in range(N):
        if dt[i] > 0:
            # 1. Update Orientation Filter
            iekf_filter.prediction(omega[i].reshape((3,)), dt[i])
            iekf_filter.correction(accel[i], gravity)
            
            # 2. Perform Dead Reckoning
            # Rotate body acceleration to world frame and subtract gravity
            a_world = (iekf_filter.X @ accel[i]) - gravity
            
            # Double integration
            p = p + (v * dt[i]) + (0.5 * a_world * (dt[i]**2))
            v = v + (a_world * dt[i])

            # 3. ZUPT (Zero Velocity Update) Logic
            accel_norm = np.linalg.norm(accel[i])
            gyro_norm = np.linalg.norm(omega[i])
            
            # If the foot is planted (acceleration is ~gravity, gyro is ~0)
            if abs(accel_norm - 9.81) < ACCEL_THRESHOLD and gyro_norm < GYRO_THRESHOLD:
                v = np.zeros(3)  # Reset velocity to 0 to kill drift!

        states_rot[i+1] = iekf_filter.X
        positions[i+1] = p
    
    # Convert rotation matrices to euler angles
    states_euler = np.zeros((N+1, 3))
    for i, rot in enumerate(states_rot):
        r = R.from_matrix(rot)
        states_euler[i] = r.as_euler('zyx', degrees=True)
        
    return states_euler, positions

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, 'data', 'imu_data_foot_raw.csv')
    
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
    else:
        print("Loading data...")
        data = riekf_load_data(file_path)
        
        print("Running RIEKF Attitude Filter & Dead Reckoning with ZUPT...")
        filter = right_iekf()
        states_euler, positions = ahrs_riekf(filter, data)
        
        # --- Plot 1: Euler Angles ---
        plt.figure(figsize=(10, 5))
        plt.plot(states_euler[:, 0], label='Yaw (Z)', color='blue', alpha=0.7)
        plt.plot(states_euler[:, 1], label='Pitch (Y)', color='green', alpha=0.7)
        plt.plot(states_euler[:, 2], label='Roll (X)', color='red', alpha=0.7)
        plt.title('Estimated IMU Orientation')
        plt.xlabel('Time Step')
        plt.ylabel('Angle (Degrees)')
        plt.legend()
        plt.grid(True)
        plt.show()

        # --- Plot 2: 2D Trajectory (X and Y) ---
        plt.figure(figsize=(8, 8))
        x = positions[:, 0]
        y = positions[:, 1]
        
        plt.plot(x, y, label='2D Path (Top-Down)', color='darkcyan', linewidth=2)
        plt.scatter(x[0], y[0], color='green', marker='o', label='Start', s=100)
        plt.scatter(x[-1], y[-1], color='red', marker='x', label='End', s=100)
        
        plt.xlabel('X Position (meters)')
        plt.ylabel('Y Position (meters)')
        plt.title('IMU 2D Trajectory with ZUPT')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.axis('equal') 
        plt.show()