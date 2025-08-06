from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    node1 = Node(
        package = 'submarine_pkg',
        executable = 'imu_publisher',
        name='imu_publisher',
        output='screen'
    )
    
    node2 = Node(
        package = 'submarine_pkg',
        executable = 'websocket_bridge',
        name='websocket_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        node1,
        node2
    ])