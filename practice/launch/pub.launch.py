from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="practice",
        executable="test",
        name="logger"
    )

    subscribe_node = Node(
        package="practice",
        executable="sub",
        name="listener"
    )

    ld.add_action(publisher_node)
    ld.add_action(subscribe_node)
    return ld