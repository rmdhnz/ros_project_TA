#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from turtlesim.msg import Pose
import math
class PoseSubscriber(Node) : 
    def __init__(self):
        super().__init__("pose_subscriber")
        self.get_logger().info("Pose Subscriber has been started and update...")
        self.pose_sub =  self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.coord_x = []
        self.coord_y = []
        self.yaw = []
    
    def pose_callback(self,pose:Pose) : 
        x = pose.x
        y = pose.y
        yaw_angle = math.radians(pose.theta)
        vx = pose.linear_velocity
        vyaw =  pose.angular_velocity
        self.get_logger().info("Position : [{} , {}]\nOrientation : {:.3f}\nVx =  {}\nVyaw  = {}\n".format(x,y,yaw_angle,vx,vyaw))
        self.coord_x.append(x)
        self.coord_y.append(y)
        self.yaw.append(yaw_angle)
    
def main(args=None):
    try : 
        rclpy.init(args=args)
        node = PoseSubscriber()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt :
        plt.plot(node.yaw)
        plt.show()

