#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_interfaces.action import CountUntil
from rclpy.action.client import ClientGoalHandle
class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.get_logger().info('Count until client has been started...')
        self.count_until_client = ActionClient(self,CountUntil,"count_until")
    
    def send_goal(self,target_number,period): 
        self.count_until_client.wait_for_server()
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        self.get_logger().info('Sending goal...')
        self.count_until_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self,future) : 
        self.goal_handle:ClientGoalHandle = future.result()
        if self.goal_handle.accepted : 
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self,future) : 
        result = future.result().result
        self.get_logger().info(f'Result : {result.reached_number}')


def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = CountUntilClient()
        mynode.send_goal(int(sys.argv[1]),float(sys.argv[2])) 
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')