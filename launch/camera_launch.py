import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_camera_wrapper'
    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'camera_params.yaml')
    
    return LaunchDescription([
        Node(
            package='kit_camera_wrapper',
            executable='camera_node',
            name='kit_camera',
            parameters=[params_file]
        )
    ])