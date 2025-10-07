# submarine_bringup.launch.py
# Bringup for:
#  - submarine_pkg/imu_publisher
#  - submarine_pkg/websocket_bridge
#  - Camera (choose: gscam OR v4l2_camera)
#  - web_video_server (HTTP MJPEG on port 8080)
# Comments without Polish diacritics.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # -------- Arguments --------
    use_gscam_arg = DeclareLaunchArgument(
        'use_gscam', default_value='true',
        description='If true use gscam, else use v4l2_camera'
    )

    device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='V4L2 device path'
    )

    width_arg  = DeclareLaunchArgument('width',  default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')
    fps_arg    = DeclareLaunchArgument('fps',    default_value='30')

    # Default gscam pipeline for YUYV cameras (like PS3 Eye)
    gscam_cfg_arg = DeclareLaunchArgument(
        'gscam_config',
        default_value=(
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=RGB'
        ),
        description='GStreamer pipeline string for gscam (RGB at the end)'
    )

    # ---- Nodes: IMU + WebSocket bridge (your package) ----
    imu_node = Node(
        package='submarine_pkg',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )

    ws_node = Node(
        package='submarine_pkg',
        executable='websocket_bridge',
        name='websocket_bridge',
        output='screen'
    )

    # ---- Camera backend A: gscam (default) ----
    # gscam reads GSCAM_CONFIG from env and publishes /camera/image_raw by default (in most builds).
    gscam_env = SetEnvironmentVariable(
        name='GSCAM_CONFIG',
        value=LaunchConfiguration('gscam_config')
    )

    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gscam'))
        # If your build publishes on /image_raw instead of /camera/image_raw
        # you can add remappings:
        # remappings=[('image_raw', '/camera/image_raw'),
        #            ('camera_info', '/camera/camera_info')]
    )

    # ---- Camera backend B: v4l2_camera (fallback) ----
    # Use YUYV input and convert to rgb8 to avoid MJPEG issues seen earlier.
    v4l2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size':   [LaunchConfiguration('width'), LaunchConfiguration('height')],
            'pixel_format': 'YUYV',      # safer than MJPG for your case
            'output_encoding': 'rgb8',   # produces sensor_msgs/Image with encoding=rgb8
            'frame_rate': LaunchConfiguration('fps')  # NOTE: key is frame_rate
        }],
        # publish under standard /camera/* names
        remappings=[
            ('image_raw',   '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        condition=UnlessCondition(LaunchConfiguration('use_gscam'))
    )

    # ---- web_video_server ----
    wvs_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        # args
        use_gscam_arg, device_arg, width_arg, height_arg, fps_arg, gscam_cfg_arg,
        # your nodes
        imu_node,
        ws_node,
        # camera
        gscam_env,
        gscam_node,
        v4l2_node,
        # http mjpeg
        wvs_node
    ])
