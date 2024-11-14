#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class EKFSubscriber(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')
        self.get_logger().info('EKF Fusion Node has been started & update...')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Topik keluaran dari EKF
            self.odom_callback,
            10
        )
        self.x_pos = []
        self.y_pos = []

    def odom_callback(self, msg:Odometry):
        # Ambil data posisi
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation

        # Tampilkan posisi dan orientasi yang sudah dikoreksi
        self.get_logger().info(f'Posisi: x={x:.2f}, y={y:.2f}')
        self.x_pos.append(x)
        self.y_pos.append(y)

def main(args=None):
    try : 
        rclpy.init(args=args)
        ekf_subscriber = EKFSubscriber()
        rclpy.spin(ekf_subscriber)
        ekf_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt :
        plt.subplot(2,1,1)
        plt.plot(ekf_subscriber.x_pos)
        plt.title("Posisi X")
        plt.xlabel("Time")
        plt.ylabel('Posisi')
        plt.grid(True)

        plt.subplot(2,1,2)
        plt.plot(ekf_subscriber.y_pos)
        plt.title("Posisi Y")
        plt.xlabel("Time")
        plt.ylabel('Posisi')
        plt.grid(True)
        plt.show()

        ekf_subscriber.get_logger().info(f"MSE (X) : {np.mean(ekf_subscriber.x_pos):.4f}")
        ekf_subscriber.get_logger().info(f"MSE (Y) : {np.mean(ekf_subscriber.y_pos):.4f}")

if __name__ == '__main__':
    main()

