#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from robot_interfaces.srv import Moving

class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server')
        self.get_logger().info('Move Server new has been started...')
        self.srv = self.create_service(Moving, 'move_to_x', self.move_to_x_callback)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()
        self.pose_now = None
    

    def pose_callback(self, msg:Pose):
        self.current_pose = msg
        self.pose_now = self.current_pose.x
        self.get_logger().info('Coordinat X = [{}]'.format(self.pose_now))

    def move_to_x_callback(self, request, response):
        target_x = request.x
        msg = Twist()
        while self.current_pose and abs(self.current_pose.x - target_x) > 0.1:
            # self.get_logger().info('Received message to move turtle to X = {} from X = {}'.format(target_x,self.pose_now))
            msg.linear.x = 1.0 if self.current_pose.x < target_x else -1.0
            self.publisher_.publish(msg)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        response.success = True
        response.message = f"Turtle moved to x: {target_x}"
        return response
    
    def match_pos(a,b,tolerance=0.1) -> bool : 
        return abs(a-b) <=tolerance

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MoveServer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt :
        print("Thanks")

if __name__ == '__main__':
    main()
