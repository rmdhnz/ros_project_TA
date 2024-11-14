#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
class DrawCircle(Node): 
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer = self.create_timer(0.5,self.send_velocity)
        self.get_logger().info("Drawing circle has been started...")
    
    def send_velocity(self) : 
        msg = Twist()
        if int(sys.argv[1])==0 :
            msg.linear.x = 1.0
        elif int(sys.argv[1])==1:
            msg.linear.y=1.0
        else : 
            msg.linear.x = 1.0
            msg.linear.y = 1.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node =  DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()