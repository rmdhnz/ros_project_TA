#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations
import pandas as pd
class ImuFiltered(Node):
    def __init__(self):
        super().__init__('imu_filtered_node')
        self.get_logger().info('Imu Filtered Node has been started ...')
        self.subs = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.imu_filterd_callback,
            10
        )
        self.data_logger = {
            "x" : [],
            "y" : [],
            "yaw" : [],
        }
    
    def imu_filterd_callback(self, msg:Odometry):
        x = msg.pose.pose.position.x+5
        y = msg.pose.pose.position.y-5
        quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.get_logger().info(f'Filtered Data\nX : {x}\nY : {y}\nYaw : {yaw}\n')
        self.data_logger["x"].append(x)
        self.data_logger["y"].append(y)
        self.data_logger["yaw"].append(yaw)



def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImuFiltered()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        D_logger = pd.DataFrame(node.data_logger)
        D_logger.to_csv("data/imu_odom_scene_1.csv",index=False)
        plt.figure(1)
        plt.plot(node.data_logger["x"],node.data_logger["y"])
        plt.title("Data posisi X,Y")
        plt.xlabel("X-position")
        plt.ylabel("Y-position")
        plt.grid(True)
        plt.show()