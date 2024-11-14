#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleMove(Node) :  
    def __init__(self):
        super().__init__('turtle_move')
        self.get_logger().info('Turtle move has been started...')
        self.cmd_vel = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.pose = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
    
    def pose_callback(self,msg: Pose) : 
        pass

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = TurtleMove()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('')