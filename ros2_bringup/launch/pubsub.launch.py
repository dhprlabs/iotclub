from launch_ros.actions import Node 
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package='ros2_tut',
        executable='publisher_oop',
        name='publisher_node',
    )

    subscriber_node = Node(
        package='ros2_tut',
        executable='subscriber_oop',
        name='subscriber_node',
    )

    publisher_node_2 = Node(
        package='ros2_tut',
        executable='publisher_oop',
        name='pub_node_2',
        remappings=[
            ('publisher_node', 'pub_node_2')
        ],
        parameters=[{"param_from_launch": "param_value"}],
    )

    subscriber_node_2 = Node(
        package='ros2_tut',
        executable='subscriber_oop',
        name='sub_node_2',
        remappings=[
            ('publisher_node', 'pub_node_2')
        ]
    )

    ld.add_action(publisher_node)
    ld.add_action(publisher_node_2)
    ld.add_action(subscriber_node)
    ld.add_action(subscriber_node_2)
    return ld