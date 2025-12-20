from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_ctrl_ros2',
            executable='speed_ctrl_node.py',
            name='speed_ctrl_node',
            output='screen'
        )
    ])
