import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import expm
import matplotlib.animation as animation
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

def motion_model(R_mat, omega, dt):
    """
    @param  R:      State variable (3, 3)
    @param  omega:  gyroscope reading (3,)
    @param  dt:     time step
    @return R_pred: predicted state variable (3, 3)
    """
    omega_hat = wedge(omega * dt)
    R_pred = R_mat @ expm(omega_hat) 
    return R_pred

def measurement_Jacobain(g):
    """
    @param  g: gravity (3,)
    @return H: measurement Jacobain (3, 3)
    """
    H = wedge(g)
    return H

class right_iekf:

    def __init__(self):
        self.Phi = np.eye(3)               # state transtion matrix
        self.Q = 1e-4*np.eye(3)            # gyroscope noise covariance
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
    
    # data['gravity'] = np.array([0.0, 0.0, 9.81])
    data['euler_gt'] = None 
    
    return data

def ahrs_riekf(iekf_filter, data):
    accel = data['accel']
    omega = data['omega']
    dt = data['dt']
    N = data['accel'].shape[0]

    # calibrate the resting biases TODOL determine the Hz and number of samples needed for a good calibration
    calib_samples = 150 # about 3 seconds of data at 50Hz (ble is roughlt this)
    gyro_bias = np.mean(omega[:calib_samples], axis=0)
    g_ref = np.mean(accel[:calib_samples], axis=0)
    g_mag = np.linalg.norm(g_ref)
    
    gravity = np.array([0.0, 0.0, g_mag]) 
    omega_corrected = omega - gyro_bias

    # Calculate magnitude arrays
    accel_mag = np.linalg.norm(accel, axis=1)
    gyro_mag = np.linalg.norm(omega_corrected, axis=1)
    
    # Use pandas rolling std/mean to capture the ENTIRE footstep, including impacts.
    accel_std = pd.Series(accel_mag).rolling(window=15, center=True).std().fillna(0).values
    gyro_smooth = pd.Series(gyro_mag).rolling(window=15, center=True).mean().fillna(0).values
    
    ACCEL_STD_THRESH = 0.5  # variance threshold
    GYRO_THRESH = 0.4
    
    is_stationary_array = (accel_std < ACCEL_STD_THRESH) & (gyro_smooth < GYRO_THRESH)

    states_rot = np.zeros((N+1, 3, 3))
    states_rot[0] = iekf_filter.X
    
    positions = np.zeros((N+1, 3))
    v = np.zeros(3) 
    p = np.zeros(3) 
    
    for i in range(N):
        if dt[i] > 0:
            is_stationary = is_stationary_array[i]

            # predict Orientation
            iekf_filter.prediction(omega_corrected[i], dt[i])
            
            if is_stationary or i < calib_samples:
                iekf_filter.correction(accel[i], gravity)
            
            # dead reckoning algo
            a_world = (iekf_filter.X @ accel[i]) - gravity
            
            if is_stationary:
                # faster decay prevents the lingering backward drag
                v = v * 0.3
                a_world = np.zeros(3) 
            else:
                v = v + (a_world * dt[i])
                
            v[2] *= 0.90 

            p = p + (v * dt[i])

        states_rot[i+1] = iekf_filter.X
        positions[i+1] = p
        
    states_euler = np.zeros((N+1, 3))
    from scipy.spatial.transform import Rotation as R
    for i, rot in enumerate(states_rot):
        r = R.from_matrix(rot)
        states_euler[i] = r.as_euler('zyx', degrees=True)
        
    return states_euler, positions

def init():
    """
    Initialize the animation by setting empty data for the line and current point.
    """
    anim_line.set_data([], [])
    current_point.set_data([], [])
    return anim_line, current_point


def update(frame):
    """
    Update the animation for each frame.
    @param frame: the current frame index
    """
    
    anim_line.set_data(x_rot[300:frame], y_rot[300:frame])
    current_point.set_data([x_rot[frame]], [y_rot[frame]])
    return anim_line, current_point
        
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, 'data/imu_data_straight_line.csv') 
    
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
    else:
        data = riekf_load_data(file_path)
        
        filter = right_iekf()
        states_euler, positions = ahrs_riekf(filter, data)
        
        x = positions[:, 0]
        y = positions[:, 1]
        
        # smooth out the jagged  path
        from scipy.signal import savgol_filter
        window = min(51, len(x) // 2 * 2 + 1)
        x_smooth = savgol_filter(x, window, 3)  
        y_smooth = savgol_filter(y, window, 3)
        
        # align the data to be facing forward
        dist = np.sqrt(x_smooth**2 + y_smooth**2)
        idx_first_leg = np.argmax(dist > 0.5)  
        if idx_first_leg > 0:
            vec = np.array([x_smooth[idx_first_leg] - x_smooth[0], y_smooth[idx_first_leg] - y_smooth[0]])
            angle = np.arctan2(vec[1], vec[0])
            
            cos_a, sin_a = np.cos(-angle), np.sin(-angle)
            x_rot = x_smooth * cos_a - y_smooth * sin_a
            y_rot = x_smooth * sin_a + y_smooth * cos_a
        else:
            x_rot, y_rot = x_smooth, y_smooth
        
        img_dir = os.path.join(script_dir, 'img')
        
        fig, ax = plt.subplots(figsize=(10, 8))
        
        static_line = ax.plot(x_rot, y_rot, label='Path', color='darkcyan', linewidth=2.5)[0]
        start_marker = ax.scatter(x_rot[0], y_rot[0], color='green', marker='*', label='Start', s=200, zorder=5)
        end_marker = ax.scatter(x_rot[-1], y_rot[-1], color='red', marker='X', label='End', s=150, zorder=5)
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('IMU Dead Reckoning with ZUPT')
        ax.legend()
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.axis('equal') 
        
        # save graph as image
        img_path = os.path.join(img_dir, 'line_trajectory.png')
        fig.savefig(img_path, dpi=300)
        static_line.remove()
        end_marker.remove()
        
        # create GIF animation of the trajectory being traced out
        
        # Create empty elements that we will update in the animation loop
        anim_line, = ax.plot([], [], color='darkcyan', linewidth=2.5, label='Path')
        current_point, = ax.plot([], [], 'o', color='orange', markersize=8, zorder=6, label='Current Position')
        
        ax.legend()

        frame_step = 5 
        frames = range(0, len(x_rot), frame_step)

        anim = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True)
        
        gif_path = os.path.join(img_dir, 'line_trajectory.gif')
        
        anim.save(gif_path, writer='pillow', fps=15)
        
        plt.show()