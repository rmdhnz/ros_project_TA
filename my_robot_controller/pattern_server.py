import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Pattern
import time

class PatternServer(Node) : 
    def __init__(self):
        super().__init__("pattern_server")
        self.get_logger().info('Pattern server has been started...')
        self.srv = self.create_service(Pattern,"pattern",self.callback_service)

    def callback_service(self,request,response):
        self.get_logger().info(f'Received request to start with {request.start_number}')
        counter = 0
        request.current_number = request.start_number
        while request.current_number!=1 : 
            counter+=1
            self.get_logger().info('Number pattern : {}'.format(request.current_number))
            if request.current_number%2==0 : 
                request.current_number = int(request.current_number/2)
            else : 
                request.current_number = request.current_number*3 + 1
            time.sleep(request.interval)
        response.final_number = counter
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = PatternServer()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')