#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
data_odom = pd.read_csv('data/data_odom_raw_scene_1.csv')
data_imu = pd.read_csv("data/imu_odom_scene_1.csv")
imu_jelek = pd.read_csv("data/data_posisi_imu.csv")
fusion_data = pd.read_csv("data/imu_odom_lidar_scene_1.csv")
data_path = pd.read_csv("data/desired_path.csv")
x_desired = data_path["x"]
y_desired = data_path["y"]

x_odom = data_odom["x"]
y_odom = data_odom["y"]
x_imu = data_imu["x"]
y_imu = data_imu["y"]
print(data_imu)
plt.plot(fusion_data["x"],fusion_data["y"],label="Fusion",color="magenta")
plt.plot(x_imu, y_imu,label="IMU")
plt.plot(x_odom, y_odom,label="Odometry")
plt.plot(x_desired, y_desired,label="path")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend()
plt.show()