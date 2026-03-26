import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class Trajectory_InEKF:
    def __init__(self):
        # The state matrix X is an element of SE_2(3). 
        # It is a 5x5 matrix holding Rotation (3x3), Velocity (3x1), and Position (3x1).
        self.X = np.eye(5) 
        
        # Gravity vector in the world frame (assuming Z is pointing up)
        self.g = np.array([0.0, 0.0, -9.81]) 

    def predict(self, accel_body, dt, quat):
        """
        Propagates the state forward in time using IMU kinematics.
        """
        # 1. Convert Quaternion to Rotation Matrix
        # Note: SciPy expects quaternions in [x, y, z, w] format. 
        # Assuming your q0 is 'w' (scalar), we rearrange to [q1, q2, q3, q0].
        # If your q3 is 'w', change this to [quat[0], quat[1], quat[2], quat[3]].
        rot_mat = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        
        # 2. Extract current state variables
        R_curr = self.X[:3, :3]
        v_curr = self.X[:3, 3]
        p_curr = self.X[:3, 4]

        # 3. Transform body acceleration to the world frame and remove gravity
        accel_world = (R_curr @ accel_body) #+ self.g
        
        # 4. Integrate to get new position and velocity (Kinematic equations)
        p_new = p_curr + (v_curr * dt) + (0.5 * accel_world * (dt**2))
        v_new = v_curr + (accel_world * dt)

        # 5. Update the state matrix with the new values
        self.X[:3, :3] = rot_mat
        self.X[:3, 3] = v_new
        self.X[:3, 4] = p_new

    def get_position(self):
        # Return a copy to prevent accidentally modifying the state history
        return self.X[:3, 4].copy()
        


def main():
    # 1. Load the Data
    # Ensure your CSV is named exactly 'imu_data.csv' and is in the same folder as this script.
    file_name = 'imu_data.csv'
    try:
        # We assume the columns are: timestamp, q0, q1, q2, q3, ax, ay, az
        # If your CSV has a header row (like "time, q0, ..."), pandas handles it automatically.
        df = pd.read_csv(file_name)
        data = df.values
    except FileNotFoundError:
        print(f"Error: '{file_name}' not found. Please ensure the file is in the same directory.")
        return

    print(f"Successfully loaded {len(data)} rows of IMU data. Processing trajectory...")

    # 2. Initialize the Filter and Storage Arrays
    filter = Trajectory_InEKF()
    trajectory = []
    
    # Store the initial position (0, 0, 0)
    trajectory.append(filter.get_position())

    # 3. Run the Dead Reckoning Loop
    for i in range(1, len(data)):
        # Calculate time difference between current and previous sample
        dt = data[i, 0] - data[i-1, 0]
        
        # Skip anomalous timestamps (e.g., duplicate rows where dt = 0)
        if dt <= 0:
            continue
        
        # Extract quaternion [q0, q1, q2, q3] and acceleration [ax, ay, az]
        quat = data[i, 1:5]
        accel = data[i, 5:8]
        
        # Run the prediction step
        filter.predict(accel, dt, quat)
        
        # Record the updated position
        trajectory.append(filter.get_position())

    # Convert the list of positions into a NumPy array for easier slicing during plotting
    trajectory = np.array(trajectory)

    print("Processing complete. Generating plots...")

    # 4. Plot the Results
    fig = plt.figure(figsize=(14, 6))

    # Plot 1: Top-Down View (X vs Y)
    ax1 = fig.add_subplot(121)
    ax1.plot(trajectory[:, 0], trajectory[:, 1], color='blue', linewidth=2, label='Estimated Path')
    ax1.plot(trajectory[0, 0], trajectory[0, 1], 'go', label='Start') # Green dot for start
    ax1.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', label='End') # Red dot for end
    ax1.set_title("Top-Down Trajectory (X-Y Plane)")
    ax1.set_xlabel("X Position (meters)")
    ax1.set_ylabel("Y Position (meters)")
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()
    ax1.axis('equal') # Ensures the scale is 1:1 so turns don't look distorted

    # Plot 2: Full 3D View
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='darkorange', linewidth=2)
    ax2.plot([trajectory[0, 0]], [trajectory[0, 1]], [trajectory[0, 2]], 'go')
    ax2.plot([trajectory[-1, 0]], [trajectory[-1, 1]], [trajectory[-1, 2]], 'ro')
    ax2.set_title("3D Trajectory")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_zlabel("Z (m)")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

