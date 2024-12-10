#!/usr/bin/env python3
import tf_transformations
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import math
# Load reference path from CSV

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw (rotation around Z-axis).
    Args:
        qx, qy, qz, qw: Components of the quaternion.
    Returns:
        float: Yaw angle in radians.
    """
    _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
    return yaw

reference_path = pd.read_csv("data/desired_path_scene_2.csv")  # Ganti dengan nama file Anda
fusion_data = pd.read_csv("data/data_fusion/ekf_fusion_data_scene_2.csv")
imu_data = pd.read_csv("data/data_imu/data_imu_mechanization.csv") 
odom_data = pd.read_csv("data/data_odom/data_odom_raw_scene_2.csv")
reference_path = reference_path.sort_values(by=['x', 'y']).reset_index(drop=True)
# Extract x and y coordinates
x_coords = reference_path['x'].values
y_coords = reference_path['y'].values
# Calculate cumulative distances along the path
distances = np.cumsum(np.sqrt(np.diff(x_coords, prepend=x_coords[0])**2 +
                              np.diff(y_coords, prepend=y_coords[0])**2))

# Create interpolation functions for x and y
interp_x = interp1d(distances, x_coords, kind='linear')
interp_y = interp1d(distances, y_coords, kind='linear')
# Generate evenly spaced distances
target_points = len(imu_data)  # Jumlah poin yang diinginkan
target_distances = np.linspace(0, distances[-1], target_points)

# Interpolate x and y coordinates
interpolated_x = interp_x(target_distances)
interpolated_y = interp_y(target_distances)

# Save interpolated path to CSV
interpolated_path = pd.DataFrame({
    'x': interpolated_x,
    'y': interpolated_y,
})
interpolated_path.to_csv("interpolated_path.csv", index=False)
yaw_angle = [0]
v_desired_x = [0]
dt = 0.01
v_desired_y = [0]
for i in range(1, len(interpolated_x)) : 
        v_desired_x.append(v_desired_x[i-1] + (interpolated_x[i]-interpolated_x[i-1])/dt)
        v_desired_y.append(v_desired_y[i-1] + (interpolated_y[i]-interpolated_y[i-1])/dt)
        yaw_angle.append(math.atan(v_desired_y[i]/v_desired_x[i]))
print("Jumlah data interpolated :",len(interpolated_path))
print("Jumlah data fusion:",len(fusion_data))
print(interpolated_x)
mse_x_fusion = np.mean((interpolated_path['x'] - fusion_data['x'])**2)
mse_y_fusion = np.mean((interpolated_path['y'] - fusion_data['y'])**2)
mse_x_odometry = np.mean((interpolated_path['x'] - odom_data['x'])**2)
mse_y_odometry = np.mean((interpolated_path['y'] - odom_data['y'])**2)
mse_x_imu = np.mean((interpolated_path['x'] - imu_data['x'])**2)
mse_y_imu = np.mean((interpolated_path['y'] - imu_data['y'])**2)

#print semua mse
print(f"MSE x Fusion: {mse_x_fusion}")
print(f"MSE y Fusion: {mse_y_fusion}")
print(f"MSE x Odometry: {mse_x_odometry}")
print(f"MSE y Odometry: {mse_y_odometry}")
print(f"MSE x IMU: {mse_x_imu}")
print(f"MSE y IMU: {mse_y_imu}")
# Plot original and interpolated paths


plt.figure(figsize=(10, 6))
plt.subplot(2,2,1)
plt.plot(fusion_data['x'], fusion_data['y'],label="Fusion data")
plt.plot(odom_data['x'], odom_data['y'],label="Odometry data")
plt.plot(imu_data['x'], imu_data['y'],label="IMU data")
plt.plot(interpolated_x, interpolated_y, '-', label="RG")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position Comparison")
plt.grid()

plt.subplot(2,2,2)
plt.plot(fusion_data['x'], label="Fusion data")
plt.plot(odom_data['x'],label="Odometry data")
plt.plot(imu_data['x'], label="IMU data")
plt.plot(interpolated_x,'-', label="RG")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position-X Comparison")
plt.grid()

plt.subplot(2,2,3)
plt.plot( fusion_data['y'],label="Fusion data")
plt.plot( odom_data['y'],label="Odometry data")
plt.plot( imu_data['y'],label="IMU data")
plt.plot(interpolated_y, '-', label="RG")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position-Y Comparison")
plt.grid()

plt.subplot(2,2,4)
plt.plot( fusion_data['yaw'],label="Fusion data")
plt.plot( odom_data['yaw'],label="Odometry data")
plt.plot( imu_data['yaw'],label="IMU data")
plt.plot(yaw_angle,label="Yaw angle")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Yaw Comparison")
plt.grid()
plt.tight_layout()
plt.show()

