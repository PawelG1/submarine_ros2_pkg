from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node 1: imu_publisher - standardowy node z pakietu submarine_pkg
    node1 = Node(
        package='submarine_pkg',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )

    # Node 2: websocket_bridge - standardowy node z pakietu submarine_pkg
    node2 = Node(
        package='submarine_pkg',
        executable='websocket_bridge',
        name='websocket_bridge',
        output='screen'
    )

    # Node 3: v4l2_camera - ustawienia domyslne dostosowane do typowej kamery USB/PS3
    # - video_device: '/dev/video0' (zmien na /dev/video1 jesli trzeba)
    # - pixel_format: 'MJPG' (kompresja MJPEG, zazwyczaj lepsze fps)
    # - image_size: [640, 480] (bezpieczna, szybka rozdzielczosc)
    # - output_encoding: 'rgb8' (konwersja do RGB, zgodna z wiekszoscia konsumentow obrazu)
    # - framerate: 30.0
    cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'MJPG',
            'image_size': [640, 480],
            'output_encoding': 'rgb8',
            'framerate': 30.0
        }],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ]
    )

    # Node 4: web_video_server - HTTP streaming (port 8080)
    wvs_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    # zwracamy LaunchDescription z wszystkimi node'ami
    return LaunchDescription([
        node1,
        node2,
        cam_node,
        wvs_node
    ])
