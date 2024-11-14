#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist

class TesDrive(Node) : 
    def __init__(self):
        super().__init__("test_drive")
        self.get_logger().info('Testing drive is ready...')
        self.pub =  self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer = self.create_timer(1.0,self.send_command)
    
    def send_command(self) : 
        msg = Twist()
        for i in range(1,len(sys.argv)) : 
            if sys.argv[i] == "go" : 
                msg.linear.x = 1.0
            elif sys.argv[i] == "back" :
                msg.linear.x = -1.0
            elif sys.argv[i] == "left" :
                msg.linear.y = 1.0
            elif sys.argv[i] == "right" :
                msg.linear.y = -1.0
            elif sys.argv[i] == "counterclockwise"  : 
                msg.linear.x=0.0
                msg.linear.y=0.0
                msg.angular.z = 1.0
            elif sys.argv[i] == "clockwise"  : 
                msg.linear.x=0.0
                msg.linear.y=0.0
                msg.angular.z = -1.0
        self.pub.publish(msg)
def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = TesDrive()
        rclpy.spin(mynode)  
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('')