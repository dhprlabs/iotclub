import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__(
            "publisher_node_param", 
            # To allow undeclared parameters...
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )
        self.declare_parameter("published_text", "Param Hellp")
        self.declare_parameter("time_period", 1.0)
        self.publisher = self.create_publisher(String, 'publisher_node', 10)
        self.timer_period = self.get_parameter("time_period").value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = String()
        self.i = 0
        
    def timer_callback(self):
        self.text = self.get_parameter("published_text").value
        self.msg.data = f"{self.text}: {self.i}"
        self.i += 1
        self.get_logger().info(f"Publishing: {self.msg.data}")
        self.publisher.publish(msg=self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()