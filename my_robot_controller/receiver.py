#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiverNode(Node):
    def __init__(self):
        super().__init__("receiver_node")
        self.get_logger().info("receiver node has been created")
        self.sub = self.create_subscription(String,'link',self.receiver_callback,10)
    
    def receiver_callback(self,msg)  : 
        num = int(msg.data)
        if num%2==0 : 
            self.get_logger().info("Angka {} adalah genap".format(num))
        else: 
            self.get_logger().info("Angka {} adalah ganjil".format(num))
        

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()