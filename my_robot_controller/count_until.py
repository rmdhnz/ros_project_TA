#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.action import CountUntil
from rclpy.action.server import ServerGoalHandle
from rclpy.action import ActionServer
import time

class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.get_logger().info('Action count has been started...')
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            "count_until",
            execute_callback=self.execute_callback)
        
    
    def execute_callback(self,goal_handle:ServerGoalHandle) :  
        target = goal_handle.request.target_number
        period = goal_handle.request.period
        self.get_logger().info('Executing the goal...')
        counter = 0
        for _ in range(target):
            counter+=1
            self.get_logger().info(f'Iteration : {counter}')
            time.sleep(period)
        goal_handle.succeed()
        result  = CountUntil.Result()
        result.reached_number = counter
        return result

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = CountUntilServer()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')
