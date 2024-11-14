#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def mse_err(data,base=None) :
    if base is None : 
        base = np.array([0 for _ in range(len(data))])
    try : 
        return np.mean((data - base)**2)
    except Exception as e : 
        print(f"Error occurred: {str(e)}")
        return None
    
imu_diam = pd.read_csv("data/imu_data_logger_diam.csv")
imu_odom_diam = pd.read_csv("data/diam_odom_imu.csv")
imu_lidar_odom_diam = pd.read_csv("data/diam_lidar_odom_imu.csv")
imu_filtered = pd.read_csv("data/imu_filtered_diam.csv")
print(imu_data_logger := pd.read_csv("imu_data_logger_scene_1.csv"))

plt.figure(1)
plt.plot(imu_data_logger["x"], imu_data_logger["y"])
plt.show()
exit()
data_lidar= pd.read_csv("data/data_lidar.csv")
print(data_lidar)
# mse_x_1 = mse_err(imu_diam["x"])
# mse_x_0 = mse_err(imu_filtered["x"])
# mse_x_2 = mse_err(imu_odom_diam["x"])
# mse_x_3 = mse_err(imu_lidar_odom_diam["x"])

# mse_y_1 = mse_err(imu_diam["y"])
# mse_y_0 = mse_err(imu_filtered["y"])
# mse_y_2 = mse_err(imu_odom_diam["y"])
# mse_y_3 = mse_err(imu_lidar_odom_diam["y"])

# print(f"MSE X imu : {mse_x_1}\nMSE x imu filt : {mse_x_0}\nMSE imu odom : {mse_x_2}\nMSE  odom lidar imu : {mse_x_3}\n")
# print(f"MSE Y imu : {mse_y_1}\nMSE y imu filt : {mse_y_0}\nMSE imu odom : {mse_y_2}\nMSE  odom lidar imu : {mse_y_3}")
exit()
data_imu = pd.read_csv("imu_data_logger_scene_1.csv")
data_odom = pd.read_csv("odometry_data_scene_1.csv")
data_fusion = pd.read_csv("fusion_data_logger_scene_1.csv")
data_filtered_imu = pd.read_csv("filtered_data_imu_tes.csv")
desired_path = pd.read_csv("desired_path.csv")

x_imu = data_imu["x"]
y_imu = data_imu["y"]

x_filt = data_filtered_imu["x"]
y_filt = data_filtered_imu["y"]

mse_x_imu = mse_err(x_imu)
mse_x_filt = mse_err(x_filt)
mse_y_imu = mse_err(y_imu)
mse_y_filt = mse_err(y_filt)

print(f"MSE X IMU : {mse_x_imu}\nMSE X Imu Filtered : {mse_x_filt}")
print(f"MSE y IMU : {mse_y_imu}\nMSE y Imu Filtered : {mse_y_filt}")

#takes desired data
x_desired = desired_path['x']
y_desired = desired_path['y']

#takes actual data from each sensor

x_fusion = data_fusion['x']
y_fusion = data_fusion['y']
yaw_fusion = data_fusion['yaw']

x_odom = data_odom['x']
y_odom = data_odom['y']
yaw_odom = data_odom['yaw']

yaw_imu = data_imu['yaw']
x_imu= data_imu['x']
y_imu = data_imu['y']
# print("MSE imu : {}".format(mse_err(x_imu)))
# print("MSE odom : {}".format(mse_err(x_odom)))
# print("MSE fusion : {}".format(mse_err(x_fusion)))

# plot desired data
plt.figure(1)
plt.plot(x_desired,y_desired,'r',label='Desired Path')
plt.plot(x_fusion,y_fusion,'b',label='Fusion Data')
plt.plot(x_odom,y_odom,'g',label='Odometry Data')
plt.plot(x_imu,y_imu,'y',label='IMU Data')
plt.title("Comparison Data")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.grid(True)
plt.legend()
plt.show()