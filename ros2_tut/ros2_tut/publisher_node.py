import rclpy
from std_msgs.msg import String
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('publisher_node')
    pub = node.create_publisher(String, 'custom_pub', 10)
    msg = String()
    i = 0
    while rclpy.ok(): # runs until ctrl + c
        i += 1
        msg.data = f'hellp: {i}'
        node.get_logger().info(f'msg: {msg.data}')
        pub.publish(msg=msg) # publish the topic

        time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__init__':
    main()