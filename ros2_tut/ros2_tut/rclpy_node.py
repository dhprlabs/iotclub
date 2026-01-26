import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rclpy_node')
    node.get_logger().info("hellp")
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__init__':
    main()