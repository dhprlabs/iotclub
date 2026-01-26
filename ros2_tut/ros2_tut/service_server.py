from ros2_interface.srv import CustomCalc

import rclpy
from rclpy.node import Node

class Service_server(Node):
    def __init__(self):
        super().__init__('my_service_server')
        self.srv = self.create_service(CustomCalc, 'custom_calc', self.custom_service_callback)

    def custom_service_callback(self, request, response):
        response.result = request.a + request.b
        self.get_logger().info(f'Result: {response.result}')
        return response
    
def main(arg=None):
    rclpy.init()
    node = Service_server()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()