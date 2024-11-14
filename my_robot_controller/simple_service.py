#!/usr/bin/env python3

from robot_interfaces.srv import Message
import rclpy
import time
from rclpy.node import Node
class SimpleService(Node):

    def __init__(self):
        super().__init__('simple_service')
        self.get_logger().info("Simple service has been created...")
        self.srv = self.create_service(Message,"chatter",self.callback_service)

    def callback_service(self,request,response) : 
        counter = 0
        for _  in range(request.target_iteration) : 
            self.get_logger().info(f'Iteration : {counter+1}')
            counter+=1
            time.sleep(request.sampling_time)
        response.iteration = counter
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_service = SimpleService()
        rclpy.spin(minimal_service)
        rclpy.shutdown()
    except KeyboardInterrupt :
        print("Thanks")
