#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import TargetCoordinates

class SenderNode(Node):
    def __init__(self):
        super().__init__("sender_node")
        self.get_logger().info("Sender node has been created...")
        self.pub = self.create_publisher(String,"link",10)
        self.timer = self.create_timer(1.0,self.callback_timer)
        self.num = 0
    
    def callback_timer(self): 
        msg  = String()
        msg.data = str(self.num)
        self.pub.publish(msg)
        self.get_logger().info('Saya mengirim angka: %s' % msg.data)
        self.num += 1

    def __del__(self) : 
        print("Thank you, have a good day!")

def main(args=None):
    rclpy.init(args=args)
    node = SenderNode()
    rclpy.spin(node)
    rclpy.shutdown()