from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    acc_package_name = 'dv_ros2_accumulation'
    acc_config_path = os.path.join(get_package_share_directory(acc_package_name), 'config', 'config.yaml')

    acc_node_name = f'{acc_package_name}_node'
    with open(acc_config_path, 'r') as file:
        acc_config = yaml.safe_load(file)[acc_package_name]['ros__parameters']
    
    cap_package_name = 'dv_ros2_capture'
    cap_config_path = os.path.join(get_package_share_directory(cap_package_name), 'config', 'config.yaml')

    cap_node_name = f'{cap_package_name}_node'
    with open(cap_config_path, 'r') as file:
        cap_config = yaml.safe_load(file)[cap_package_name]['ros__parameters']

    vis_package_name = 'dv_ros2_visualization'
    vis_config_path = os.path.join(get_package_share_directory(vis_package_name), 'config', 'config.yaml')

    vis_node_name = f'{vis_package_name}_node'
    with open(vis_config_path, 'r') as file:
        vis_config = yaml.safe_load(file)[vis_package_name]['ros__parameters']

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
            package=acc_package_name,
            executable=acc_node_name,
            name=f'{acc_node_name}_frame',
            namespace=f'{acc_node_name}_frame',
            parameters=[acc_config],
            output='screen',
            emulate_tty=True,
            remappings=[
                (f'/{acc_node_name}_frame/events', '/events'),            
            ]
        ),  
        Node(
            package=acc_package_name,
            executable=acc_node_name,
            name=f'{acc_node_name}_edge',
            namespace=f'{acc_node_name}_edge',
            parameters=[acc_config | {'accumulation_mode': "EDGE", 'event_contribution': 0.3}],
            output='screen',
            emulate_tty=True,
            remappings=[
                (f'/{acc_node_name}_edge/events', '/events'),            
            ]
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
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            parameters=[{'perspective_file': os.path.join(get_package_share_directory('dv_ros2_visualization'), 'config', 'all.perspective')}],
            emulate_tty=True,
        )
    ])