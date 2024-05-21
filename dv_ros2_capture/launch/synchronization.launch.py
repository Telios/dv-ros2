from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Master camera node will wait for other cameras to be online 
        # and will send hardware synchronization signal 
        Node(
            package='dv_ros2_capture',
            executable='dv_ros2_capture_node',
            name='master_cam',
            namespace='',
            parameters=[{'cameraName': 'DVXplorer_DXA00252', 
                         'syncDevices': ['DAVIS346_00000499', 'DVXplorer_DXA00087']}],
            output='screen'
        ),

        # Other cameras, These cameras will report their status to 
        # the master node and will not publish any data until synchronization 
        # signal is received
        # cam1
        Node(
            package='dv_ros2_capture',
            executable='dv_ros2_capture_node',
            name='cam1',
            namespace='',
            parameters=[{'cameraName': 'DVXplorer_DXA00087', 
                         'waitForSync': True}],
            output='screen'
        ),

        # cam2
        Node(
            package='dv_ros_capture',
            executable='capture_node',
            name='cam2',
            namespace='',
            parameters=[{'cameraName': 'DAVIS346_00000499', 
                         'waitForSync': True}],
            output='screen'
        ),
    ])