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
        additional_env={'GSCAM_CONFIG': "v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! \
  video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
  videoconvert ! video/x-raw,format=RGB ! \
  appsink sync=false max-buffers=1 drop=true"}
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
