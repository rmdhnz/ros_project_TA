#!/usr/bin/env python3

import rclpy
from robot_interfaces.srv import Message
from rclpy.node import Node
import sys

class SimpleClient(Node):  
    def __init__(self):
        super().__init__("client_simple")
        self.get_logger().info('Simple client has been created...')
        self.cli = self.create_client(Message,"chatter")
        while not self.cli.wait_for_service(timeout_sec=1.0)  : 
            self.get_logger().info('Waiting for service...')
        self.req  = Message.Request()
    
    def send_request(self,target_iteration,time_sampling=1.0) :
        self.req.target_iteration = target_iteration
        self.req.sampling_time = time_sampling
        return self.cli.call_async(self.req)


def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = SimpleClient()
        future = mynode.send_request(int(sys.argv[1]),float(sys.argv[2]))
        rclpy.spin_until_future_complete(mynode, future)
        mynode.get_logger().info("Goal has been reached")
        mynode.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')
