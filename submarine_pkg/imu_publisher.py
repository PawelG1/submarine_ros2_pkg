import json
import math
from smbus2 import SMBus
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from submarine_pkg.mini_imu import MiniIMU

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10) # publisher dla danych topicu IMU
        self.euler_publisher = self.create_publisher(Vector3, 'imu/euler_angles', 10) # publisher dla katow Eulera

        self.imu = MiniIMU(1)
        self.imu.load_calibration()

        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("IMU Publisher uruchomiony")

    def timer_callback(self):
        acc, gyro, _ = self.imu.read_data(scaled=True)
        #pobranie katow po filtracji z IMU
        pitch, roll, yaw = self.imu.get_all_angles()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # m/s^2 accelerometr
        g = 9.80665
        msg.linear_acceleration.x = acc[0] * g
        msg.linear_acceleration.y = acc[1] * g
        msg.linear_acceleration.z = acc[2] * g

        # gyro rad/s
        deg2rad = 3.141592 / 180.0
        msg.angular_velocity.x = gyro[0] * deg2rad
        msg.angular_velocity.y = gyro[1] * deg2rad
        msg.angular_velocity.z = gyro[2] * deg2rad

        #pomineity magnetometr
        msg.orientation_covariance[0] = -1  # oznacza "brak danych"

        #publikowanie danych na topicu IMU
        self.publisher_.publish(msg)

        #utworzenie wiadomosci Vector3 dla katow
        euler_angles = Vector3()
        euler_angles.x = pitch
        euler_angles.y = roll
        euler_angles.z = yaw
        self.euler_publisher.publish(euler_angles)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
