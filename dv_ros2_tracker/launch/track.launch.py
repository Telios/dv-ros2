from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    cap_package_name = 'dv_ros2_capture'
    cap_config_path = os.path.join(get_package_share_directory(cap_package_name), 'config', 'config.yaml')

    cap_node_name = f'{cap_package_name}_node'
    with open(cap_config_path, 'r') as file:
        cap_config = yaml.safe_load(file)[cap_package_name]['ros__parameters']

    track_package_name = 'dv_ros2_tracker'
    track_config_path = os.path.join(get_package_share_directory(track_package_name), 'config', 'config.yaml')
    
    track_node_name = f'{track_package_name}_node'
    with open(track_config_path, 'r') as file:
        track_config = yaml.safe_load(file)[track_package_name]['ros__parameters']
    
    return LaunchDescription([
        Node(
            package=cap_package_name,
            executable=cap_node_name,
            name=cap_node_name,
            parameters=[cap_config],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package=track_package_name,
            executable=track_node_name,
            name=track_node_name,
            namespace=track_node_name,
            parameters=[track_config],
            output='screen',
            remappings=[
                (f'/{track_node_name}/events', '/events'),
                (f'/{track_node_name}/camera_info', '/camera_info'),            
            ],
            emulate_tty=True,
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            emulate_tty=True,
        )
    ])