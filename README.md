# EECS 568 Final Project - Indoor Localization via IMU Dead Reckoning and ZUPT 

## Project Overview
This project implements an indoor localization system using a foot-mounted 9-DoF IMU (ICM-20948) and an ESP32 microcontroller. The system addresses the inherent challenge of sensor drift in GPS-denied environments by utilizing an Extended Kalman Filter (EKF) and specialized Zero-Velocity Updates (ZUPT) to reconstruct accurate 2D trajectories from raw inertial data.

### Results
![Straight Line Trajectory](/img/line_trajectory.png)

### Data Processing 
1. **Acquisition:** Raw accelerometer, gyroscope, and magnotometer data are sampled through the ESP32.
2. **Orientation Estimation:** Sensor data is transformed from the local body frame to a global navigation frame.
3. **Gravity Compensation:** Measured acceleration is corrected by removing the gravitational vector to isolate true linear motion.
4. **Kinematic Integration:** Acceleration is integrated to estimate velocity and position.

### Extended Kalman Filter (EKF)
To manage the quadratic growth of position error, an EKF maintains a state vector consisting of position, velocity, orientation, and sensor biases.

$$x = [p, v, q, b_a, b_g]^T$$

### Soft Zero-Velocity Updates (ZUPT)
The primary correction mechanism relies on detecting the "stance phase" of human gait. While classical ZUPT assumes absolute zero velocity, this project implements a Soft ZUPT formulation.

## Application
This system is designed for pedestrian navigation, unlike traditional mobile robots. By treating the human foot as a periodic stationary reference, the soft ZUPT algorithm provides an correction loop that maintains estimator consistency without the need for anything external devices outside the system.


## Group Members
* Nate Sochocki
* Sravya Ganti
* Max West
