import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class MySubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.msg = String()
        self.create_subscription(String, 'publisher_node', self.subscriber_callback, 10)

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()