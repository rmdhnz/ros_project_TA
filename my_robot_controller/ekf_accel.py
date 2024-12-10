#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
class ExtendedKalmanFilter:
    def __init__(self, Q, R):
        # State vector [a_x, a_y, a_z]
        self.x = np.zeros((3, 1))
        # Covariance matrix
        self.P = np.eye(3)
        # Process noise covariance
        self.Q = Q
        # Measurement noise covariance
        self.R = R

    def predict(self, A):
        # Predict state (f(x) = x, so no change in state)
        self.x = self.x
        # Predict covariance
        self.P = A @ self.P @ A.T + self.Q

    def update(self, z, H):
        # Measurement residual
        y = z - self.x
        # Measurement covariance
        S = H @ self.P @ H.T + self.R
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update state
        self.x = self.x + K @ y
        # Update covariance
        self.P = (np.eye(3) - K @ H) @ self.P

# Simulasi data percepatan linear dengan noise
data = pd.read_csv("data/data_imu/data_raw_imu_scene_2.csv")
np.random.seed(42)
true_acceleration = np.array([[0], [0], [0]])  # True acceleration (robot is stationary)
measurement_variance = 0.05  # Variance in accelerometer measurements
bias_ax = np.mean(data["ax"])
bias_ay = np.mean(data["ay"])
bias_az = np.mean(data["az"])
# data["ax"]-=bias_ax
# data["ay"]-=bias_ay
# data["az"]-=bias_az
accel_measurements = data[['ax', 'ay', 'az']].to_numpy()
# Inisialisasi EKF
Q = np.eye(3) * 0.01  # Process noise covariance
R = np.eye(3) * measurement_variance  # Measurement noise covariance
ekf = ExtendedKalmanFilter(Q, R)
# Matriks A dan H
A = np.eye(3)  # State transition Jacobian
H = np.eye(3)  # Measurement Jacobian

# Estimasi EKF
estimates = []
try :
    for z in accel_measurements:
        ekf.predict(A)
        ekf.update(z, H)
        estimates.append(ekf.x.flatten())

    estimates = np.array(estimates)
    kf_ax =  []
    kf_ay =  []
    kf_az =  []

    for i in range(len(estimates)) : 
        kf_ax.append(estimates[i][0])
        kf_ay.append(estimates[i][1])
        kf_az.append(estimates[i][2])
    # Plot hasil
    time = data["time"] # Simulate for 60 seconds
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.plot(time,accel_measurements[:,0],label="Noisy Measurement X")
    plt.plot(time,estimates[:,0],label="Filtered Accelero-X")
    plt.xlabel('Time (s)')
    plt.ylabel('X Acceleration (m/s²)')
    plt.legend()
    plt.subplot(3,1,2)
    plt.plot(time, accel_measurements[:,1],label="Noisy Measurement Y")
    plt.plot(time, estimates[:,1],label="Filtered Accelero-Y")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Acceleration (m/s²)')
    plt.legend()
    plt.subplot(3,1,3)
    plt.plot(time, accel_measurements[:,2],label="Noisy Measurement Z")
    plt.plot(time, estimates[:,2],label="Filtered Accelero-Z")
    plt.xlabel('Time (s)')
    plt.ylabel('Z Acceleration (m/s²)')
    plt.legend()

    plt.tight_layout()
    plt.show()


    dlogger = {
            "time" : data["time"],
            "ax" : data["ax"],
            "ay" : data["ay"],
            "az" : data["az"],
            "wx" : data["wx"],
            "wy" : data["wy"],
            "wz" : data["wz"],
            "or_x" : data["or_x"],
            "or_y" : data["or_y"],
            "or_z" : data["or_z"],
            "or_w" : data["or_w"],
            "kf_ax" :  kf_ax,
            "kf_ay" :  kf_ay,
            "kf_az" :  kf_az,
    }
    d = pd.DataFrame(dlogger)
    d.to_csv("data/data_imu/data_ekf_accel.csv",index=False)
except Exception as e:
    print("Error: " + str(e))
