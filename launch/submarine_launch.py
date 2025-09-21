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

    # Node 3: V4L2 camera driver (replaces gscam)
    # It grabs frames from /dev/video0 using V4L2 legacy driver (bcm2835-v4l2)
    # and publishes sensor_msgs/Image on /image_raw by default.
    # We remap to /camera/image_raw to keep a standard topic name for web_video_server.
    cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},    # which device to open
            {'image_size': [640, 480]},         # width, height
            {'pixel_format': 'YUYV'},           # source V4L2 pixel format
            {'output_encoding': 'rgb8'},        # convert to RGB for ROS consumers
            {'framerate': 30.0}                 # requested FPS
        ],
        remappings=[
            ('image', '/camera/image_raw'),     # publish on /camera/image_raw
            ('camera_info', '/camera/camera_info')
        ]
    )

    # Node 4: web_video_server for HTTP streaming
    wvs_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[
            {'port': 8080}   # change if you need a different port
        ]
    )
    
    return LaunchDescription([
        node1,
        node2,
        cam_node,
        wvs_node
    ])
