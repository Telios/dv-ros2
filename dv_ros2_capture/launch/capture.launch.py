from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    package_name = 'dv_ros2_capture'
    config_path = os.path.join(get_package_share_directory(package_name), 'config', 'config.yaml')

    node_name = f'{package_name}_node'
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)[package_name]['ros__parameters']

    return LaunchDescription([
        Node(
            package=package_name,
            executable=node_name,
            name=node_name,
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])