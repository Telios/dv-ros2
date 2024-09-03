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

    vis_package_name = 'dv_ros2_visualization'
    vis_config_path = os.path.join(get_package_share_directory(vis_package_name), 'config', 'doge_config.yaml')

    vis_node_name = f'{vis_package_name}_node'
    with open(vis_config_path, 'r') as file:
        vis_config = yaml.safe_load(file)[vis_package_name]['ros__parameters']
    
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
            package=vis_package_name,
            executable=vis_node_name,
            name=vis_node_name,
            namespace=vis_node_name,
            parameters=[vis_config],
            output='screen',
            emulate_tty=True,
            remappings=[
                (f'/{vis_node_name}/events', '/events'),            
            ]
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            emulate_tty=True,
        )
    ])