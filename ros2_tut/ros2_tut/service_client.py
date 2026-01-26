import sys
from ros2_interface.srv import CustomCalc

import rclpy
from rclpy.node import Node

class Service_client(Node):
    def __init__(self):
        super().__init__('my_service_client')
        self.client = self.create_client(CustomCalc, 'custom_calc')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting...')
        self.request = CustomCalc.Request()
        
    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        return self.client.call_async(self.request)
    
def main(args=None):
    rclpy.init(args=args)
    node = Service_client()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(node=node, future=future)
    response = future.result()
    node.get_logger().info(f'Result: {response.result}')
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()