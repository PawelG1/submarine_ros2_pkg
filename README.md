Submarine Control Package

A ROS2 package for controlling a surface/submersible boat on a Raspberry Pi, with support for the Pololu MinIMU-9 v5 IMU sensor.
 Description

This package provides a complete control system for a surface/submersible boat, enabling:

    Reading orientation data from the IMU sensor (pitch, roll, yaw)

    Transmitting telemetry data over WebSocket/TCP

    Integration with a Flutter app (in a separate repository)

    Advanced IMU data filtering with drift compensation

		Components
IMU Sensor – Pololu MinIMU-9 v5

This package supports the Pololu MinIMU-9 v5, which includes:

    LSM6DS33 – 3-axis accelerometer + gyroscope

    LIS3MDL – 3-axis magnetometer

    Communication via I2C

    Advanced filtering algorithms (complementary filter + low-pass)

ROS2 Nodes

    imu_publisher – publishes IMU data

        Topic: /imu/data_raw (sensor_msgs/Imu)

        Topic: /imu/euler_angles (geometry_msgs/Vector3)

        Rate: 50 Hz

    websocket_bridge – TCP/WebSocket bridge

        Port: 8765

        JSON-formatted telemetry data

        Rate: 20 Hz

			Hardware Requirements

    Raspberry Pi (tested on Pi 4)

    Pololu MinIMU-9 v5 connected via I2C

    I2C interface enabled on the Raspberry Pi

		Installation
1. Prepare the Raspberry Pi

# Enable I2C
sudo raspi-config
# Interfacing Options -> I2C -> Enable

# Install required system packages
sudo apt update
sudo apt install python3-pip python3-smbus i2c-tools

# Verify sensor detection (should see addresses 0x1e and 0x6b)
i2cdetect -y 1

2. Install ROS2 (if not already installed)

# Add ROS2 repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

3. Install the submarine_pkg

# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone <YOUR_REPO_URL> submarine_pkg

# Install Python dependencies
cd submarine_pkg
pip3 install smbus2

# Build the package
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select submarine_pkg

# Source the workspace
source install/setup.bash

4. Calibrate the IMU 

Before first use, you must calibrate the sensor:

# Go to the package directory
cd ~/ros2_ws/src/submarine_pkg/submarine_pkg

# Run calibration (keep the sensor still!)
python3 mini_imu.py

# Calibration takes about 40 seconds 
# A file named calibration.json will be created automatically

	Running
Launch individual nodes:

# Terminal 1 – IMU Publisher
ros2 run submarine_pkg imu_publisher

# Terminal 2 – WebSocket Bridge
ros2 run submarine_pkg websocket_bridge

Launch the entire system (recommended):

ros2 launch submarine_pkg submarine_launch.py

	Monitoring Data
Inspect ROS2 topics:

# List available topics
ros2 topic list

# View raw IMU data
ros2 topic echo /imu/data_raw

# View Euler angles
ros2 topic echo /imu/euler_angles

Test the TCP connection:

# Connect to the WebSocket bridge
telnet localhost 8765

# You should receive JSON data every 50 ms:
# {"pitch": -1.23, "roll": 0.45, "speed": 4.0, "battery_voltage": 11.8, "rudder_angle": 1.0, "yaw": 180.0}

		Configuration
Change the sampling rate:

Edit imu_publisher.py:

timer_period = 0.02  # 50 Hz (default)

Change the WebSocket port:

Edit websocket_bridge.py:

self.port = 8765  # change to another port if desired

IMU filtering parameters:

In mini_imu.py, you can adjust:

alpha = 0.95  # complementary filter (0.0–1.0)
# alpha closer to 1.0 = more gyroscope influence
# alpha closer to 0.0 = more accelerometer influence

		Flutter App Integration

This package is designed to work with a Flutter app. Telemetry data is sent as JSON over TCP on port 8765:
	see: https://github.com/PawelG1/ROSBoatControlPanel.git

{
  "pitch": -1.23,
  "roll": 0.45,
  "speed": 4.0,
  "battery_voltage": 11.8,
  "rudder_angle": 1.0,
  "yaw": 180.0
}


	Troubleshooting
IMU not detected:

# Check I2C connection
i2cdetect -y 1

# You should see:
# 0x1e – LIS3MDL magnetometer
# 0x6b – LSM6DS33 accelerometer/gyroscope

I2C permission errors:

# Add your user to the i2c group
sudo usermod -a -G i2c $USER
# Then log out and log back in

Unstable readings:

    Ensure calibration was completed

    Check I2C wiring quality

    Tune filtering parameters in mini_imu.py

WebSocket not working:

# Check if the port is in use
netstat -tulpn | grep :8765

# Run the node with debug logs
ros2 run submarine_pkg websocket_bridge --ros-args --log-level debug

 



Note: This package is a work in progress (WIP). Use at your own risk during water tests! 