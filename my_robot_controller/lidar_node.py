#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd
class LidarToCartesian(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar node has been started &  update...')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )
        self.data_logger = {
            "angle_min" : [],
            "angle_max" : [],
            "angle_increment" : [],
            "time_increment" : [],
            "scan_time" : [],
            "range_min" : [],
            "range_max" : [],
            "ranges" : [],
            "intensities" : []  # Daftar koordinat kartesian
        }  # Daftar koordinat kartesian

    def scan_callback(self, msg:LaserScan):
        # Mendapatkan data jarak dari Lidar
        ranges = np.array(msg.ranges)
        self.data_logger["ranges"].append(ranges)

        # Mendapatkan parameter sudut dari pesan LaserScan
        angle_min = msg.angle_min      # Sudut minimum
        self.data_logger["angle_min"].append(angle_min)
        angle_increment = msg.angle_increment  # Kenaikan sudut untuk setiap pembacaan
        self.data_logger["angle_increment"].append(angle_increment)
        self.data_logger["scan_time"].append(msg.scan_time)
        self.data_logger["range_min"].append(msg.range_min)
        self.data_logger["range_max"].append(msg.range_max)
        self.data_logger["angle_max"].append(msg.angle_max)
        self.data_logger["time_increment"].append(msg.time_increment)
        self.data_logger["intensities"].append(msg.intensities)  # Daftar intensitas
        # Inisialisasi array untuk menyimpan koordinat kartesian

        # Konversi ke koordinat kartesian (x, y) untuk setiap jarak yang diterima dari Lidar
        # for i, distance in enumerate(ranges):
        #     if np.isinf(distance) or np.isnan(distance):
        #         # Abaikan nilai yang tidak valid (jarak tak terhingga atau NaN)
        #         continue

        #     # Hitung sudut untuk setiap titik i
        #     angle = angle_min + i * angle_increment
            
        #     # Konversi jarak ke koordinat kartesian (x, y)
        #     x = distance * np.cos(angle)
        #     y = distance * np.sin(angle)

        # self.get_logger().info(self.data_logger)
        # Tampilkan koordinat kartesian
def main(args=None):
    try : 
        rclpy.init(args=args)
        node = LidarToCartesian()
        rclpy.spin(node)  # Jalankan node sampai dihentikan
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt : 
        dlogger = pd.DataFrame(node.data_logger)
        dlogger.to_csv("data/data_lidar/datalidar.csv",index=False)
        # dlogger_anggle_min = pd.DataFrame(node.data_logger["angle_min"])
        # dlogger_anggle_max = pd.DataFrame(node.data_logger["angle_max"])
        # dlogger_angle_increment = pd.DataFrame(node.data_logger["angle_increment"])
        # dlogger_scan_time = pd.DataFrame(node.data_logger["scan_time"])
        # dlogger_range_min = pd.DataFrame(node.data_logger["range_min"])
        # dlogger_range_max = pd.DataFrame(node.data_logger["range_max"])
        # dlogger_anggle_min.to_csv("data/data_lidar/angle_min.csv")
        # dlogger_anggle_max.to_csv("data/data_lidar/angle_max.csv")
        # dlogger_angle_increment.to_csv("data/data_lidar/angle_increment.csv")
        # dlogger_scan_time.to_csv("data/data_lidar/scan_time.csv")
        # dlogger_range_min.to_csv("data/data_lidar/range_min.csv")
        # dlogger_range_max.to_csv("data/data_lidar/range_max.csv")

if __name__ == '__main__':
    main()
