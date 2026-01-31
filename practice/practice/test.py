import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

class myNode(Node):
    def __init__(self):
        super().__init__("test")
        self.publisher = self.create_publisher(String, 'topic' , 10)
        self.timer = self.create_timer(0.5,self.timer_cb)
        self.i = 0
        self.msg = String()
        self.get_logger().info("Test node has been started.")

    def timer_cb(self):
        self.msg.data = f"Hello,world: {self.i}"
        self.i += 1
        self.get_logger().info(f'Publishing: "{self.msg.data}"')
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = myNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
