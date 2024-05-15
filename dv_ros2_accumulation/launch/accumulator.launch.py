from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'dv_ros2_accumulation'
    return LaunchDescription([
        Node(
            package=package_name,
            executable=f'{package_name}_node',
            name=f'{package_name}_node',
            parameters=[
                {'publisher_topic_image': 'image',
                 'rate': 50.0,
                 },
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])