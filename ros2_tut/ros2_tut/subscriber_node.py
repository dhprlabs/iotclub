import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('subscriber_node')

    def subs_callback(msg):
        node.get_logger().info(f'Received: {msg.data}')
    
    subscriber = node.create_subscription(String, 'publisher_node', subs_callback, 10)
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()