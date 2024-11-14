#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class DrawRectangle(Node):
    def __init__(self):
        super().__init__("draw_rectangle")
        self.get_logger().info("Drawing rectangle has been started...")
        self.publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.draw_square()
    
    def send_command(self) : 
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher.publish(msg)
    
    def draw_square(self) : 
        while True :
            self.move_straight(2.0)
            sleep(2)
            self.turn(90)
            sleep(1)
    

    def move_straight(self,time_duration) : 
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Moving straight for  %f seconds" % time_duration)
        sleep(time_duration)
        self.stop_turtle()
    
    def turn(self,angle) :
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.57
        self.publisher.publish(msg)
        self.get_logger().info("Turning for 1 second to achieve %d degree" % angle)
        sleep(1)
        self.stop_turtle()
    
    def stop_turtle(self) : 
        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Stopping turtle!")


def main(args=None):
    try : 
        rclpy.init(args=args)
        node = DrawRectangle()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt : 
        print("Thanks")