import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    node = Node(
        package='wheelctrl_ros2',
        executable = 'wheelctrl_ros2',
        name='wheelctrl_ros2',
        output='screen',
        emulate_tty = True,
        parameters=[os.path.join(
            get_package_share_directory('wheelctrl_ros2'),
            'config','er.yaml')]
    )
    
    ld.add_action(node)
    return ld
