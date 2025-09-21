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

    node3 = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_publisher',
        output='screen',
        parameters=[
            {'sync_sink': False},           # nie blokuj na sync
            {'use_gst_timestamps': True},   # bierz timestampy z GStreamera
            {'image_encoding': 'rgb8'},     # pasuje do format=RGB
            {'frame_id': 'camera_frame'}
        ],
        #mienna GSCAM_CONFIG tylko dla tego noda
        #
    )
    
    node4 = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )
    
    return LaunchDescription([
        node1,
        node2,
        node3,
        node4
    ])
