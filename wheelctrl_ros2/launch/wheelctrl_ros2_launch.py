from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node
            (
                package='wheelctrl_ros2',
                namespace= 'wheelctrl_ros2',
                executable = 'wheelctrl_ros2',
                name='wheelctrl_ros2'
            )
        ]
    )