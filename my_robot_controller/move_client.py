#!/usr/bin/env python3
# bawah -1.552 kiri -3.136 atas 1.579 kanan 0
import sys
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Moving

class MoveXClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = self.create_client(Moving, 'move_to_x')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the server...')

    def send_request(self, x):
        req = Moving.Request()
        req.x = x
        future = self.client.call_async(req)
        return future

def main(args=None):
    try:
        rclpy.init(args=args)
        client = MoveXClient()
        x = float(sys.argv[1])
        future = client.send_request(x)
        rclpy.spin_until_future_complete(client, future)

        response = future.result()
        if response.success:
            client.get_logger().info(f'Response: {response.message}')
        else:
            client.get_logger().info('Failed to move turtle.')

        client.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Thanks")

if __name__ == '__main__':
    main()
