import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Pattern
import sys
class PatternClient(Node) : 
    def __init__(self):
        super().__init__("pattern_client")
        self.get_logger().info('Pattern Client has been started...')
        self.cli = self.create_client(Pattern,"pattern")
        while not self.cli.wait_for_service(timeout_sec=1.0) : 
            self.get_logger().info('Waiting for service to start...')
        self.req = Pattern.Request()
    
    def send_request(self,start_num,interval=1.0) : 
        self.req.start_number = start_num
        self.req.interval = interval
        return self.cli.call_async(self.req)

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = PatternClient()
        if int(sys.argv[1]) <=0 : 
            mynode.get_logger().info("Starting number must be more than 0")
            exit()
        future = mynode.send_request(int(sys.argv[1]),float(sys.argv[2]))
        rclpy.spin_until_future_complete(mynode,future)
        mynode.get_logger().info("Goal has been reached for {} iterations".format(type(future.result())))
        mynode.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')