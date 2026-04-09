
# Submarine Control Package

A ROS2 package for controlling a surface/submersible boat on a Raspberry Pi, with support for the Pololu MinIMU-9 v5 IMU sensor.



## Description

This package provides a complete control system for a surface/submersible boat, enabling:

- Reading orientation data from the IMU sensor (pitch, roll, yaw)  
- Transmitting telemetry data over WebSocket/TCP  
- Integration with a Flutter app (in a separate repository)  
- Advanced IMU data filtering with drift compensation  



## Components

### IMU Sensor – Pololu MinIMU-9 v5

This package supports the Pololu MinIMU-9 v5, which includes:

- LSM6DS33 – 3-axis accelerometer + gyroscope  data
- LIS3MDL – 3-axis magnetometer  data
- Advanced filtering algorithms (complementary filter + low-pass)  

## ROS2 Nodes

1. **imu_publisher** – publishes IMU data  
   - Topic: `/imu/data_raw` (sensor_msgs/Imu)  
   - Topic: `/imu/euler_angles` (geometry_msgs/Vector3)  
   - Rate: 50 Hz  

2. **websocket_bridge** – TCP/WebSocket bridge  
   - Port: 8765  
   - JSON-formatted telemetry data  
   - Rate: 20 Hz  


## Hardware Requirements

- Raspberry Pi (tested on Pi 5)  
- Pololu MinIMU-9 v5 connected via I2C  
- I2C interface enabled on the Raspberry Pi  


## Installation

### 1. Prepare the Raspberry Pi

```bash
# Install required system packages
sudo apt update
sudo apt install python3-pip python3-smbus i2c-tools

#install curl
sudo apt install curl

#install git
sudo apt install git

#install colcon
sudo apt install colcon

#install gscam and web-video-server
sudo apt install ros-jazzy-gscam
sudo apt install ros-jazzy-web-video-server

# Enable I2C
sudo apt install i2c-tools

# Verify sensor detection (should see addresses 0x1e and 0x6b)
i2cdetect -y 1

2. Install ROS2 (if not already installed)

# Add ROS2 repository

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

## for ubuntu 24.04 use newer distro: jazzy
sudo apt install ros-jazzy-desktop
```
---

## 3. Configure Direct Ethernet Connection

This setup creates a dedicated, cable-only link between the Pi and the laptop.
Both devices can still access the internet independently via Wi-Fi.

> **Important**: Check your router's DHCP subnet first (`ip route show` on the laptop).
> The Ethernet subnet must not overlap with your Wi-Fi subnet.
> This guide uses `10.0.0.x` — safe for most home networks.

---

### Raspberry Pi 5 (Ubuntu)

Find your MAC address and existing Netplan config:

```bash
ip link show eth0        # note the MAC address
ls /etc/netplan/         # look for 50-cloud-init.yaml
```

Edit the Ethernet config file:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Replace the contents with (keep your actual MAC address):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      match:
        macaddress: "XX:XX:XX:XX:XX:XX"  # replace with your MAC
      set-name: "eth0"
      dhcp4: no
      addresses: [10.0.0.2/24]
```

> **Note**: Do not add `renderer: networkd` — it will break Wi-Fi on Ubuntu Desktop.
> Wi-Fi lives in a separate Netplan file and is unaffected.

Apply:

```bash
sudo netplan apply
# Warnings about systemd-networkd are expected on Ubuntu Desktop — not an error.
```

Verify:

```bash
ip addr show eth0
# Expected: inet 10.0.0.2/24
```

---

### Laptop – Linux (NetworkManager)

Find your Ethernet interface name:

```bash
ip link show
# Common names: eth0, enp2s0, enp3s0 — use yours below
```

Create a static profile that does not override the default Wi-Fi route:

```bash
sudo nmcli connection add \
  type ethernet \
  ifname enp2s0 \
  con-name rpi-link \
  ipv4.addresses 10.0.0.1/24 \
  ipv4.method manual \
  ipv4.never-default yes

sudo nmcli connection up rpi-link
```

The `ipv4.never-default yes` flag ensures internet traffic continues to flow via Wi-Fi.

To make the profile reconnect automatically after reboot:

```bash
sudo nmcli connection modify rpi-link connection.autoconnect yes
```

---

### Laptop – Windows

1. Open **Settings → Network & Internet → Ethernet → Edit**.
2. Set **IP assignment** to **Manual**.
3. Enter:
   - IP address: `10.0.0.1`
   - Subnet mask: `255.255.255.0`
   - Gateway: *(leave empty)*
4. Click **Save**.

---

### SSH Setup (first time only)

On the Raspberry Pi:

```bash
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

---

### Verify the Connection

From the laptop:

```bash
ping 10.0.0.2
ssh rpi5@10.0.0.2
```

From the Raspberry Pi:

```bash
ping 10.0.0.1
```

---

### Troubleshooting

| Symptom | Fix |
|---|---|
| `ping` fails | Run `ip addr show eth0` on Pi and `ip addr show enp2s0` on laptop — both must have `10.0.0.x` addresses |
| Internet stops on laptop after connecting cable | Re-run: `sudo nmcli connection modify rpi-link ipv4.never-default yes && sudo nmcli connection up rpi-link` |
| Wi-Fi stops on Pi after `netplan apply` | Check that `renderer: networkd` is NOT in `50-cloud-init.yaml` |
| SSH hangs / times out | Verify SSH is running: `sudo systemctl status ssh` |
| Settings lost after reboot (Pi) | Netplan files persist automatically — verify with `ip addr show eth0` after reboot |
| Settings lost after reboot (laptop) | Run: `sudo nmcli connection modify rpi-link connection.autoconnect yes` |

---

4. Install the package

# Navigate to your ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/PawelG1/submarine_ros2_pkg.git submarine_pkg

# Install Python dependencies
cd submarine_pkg
sudo apt install python3-smbus2

# Build the package
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select submarine_pkg

# Source the workspace
source install/setup.bash

```

# 4. Calibrate the IMU (IMPORTANT)

Before first use, you should calibrate the sensor:

### Go to the package directory
cd ~/ros2_ws/src/submarine_pkg/submarine_pkg

### Run calibration (keep the sensor still)
python3 mini_imu.py

### Calibration takes about ~40 seconds 

### A file named calibration.json will be created automatically

	

# Running
## Launch individual nodes

### Terminal 1 – IMU Publisher
	ros2 run submarine_pkg imu_publisher

### Terminal 2 – WebSocket Bridge
	ros2 run submarine_pkg websocket_bridge

# Launch the entire system (recommended)

	ros2 launch submarine_pkg submarine_launch.py

# Monitoring Data
### Inspect ROS2 topics
##### List available topics
	ros2 topic list

### View raw IMU data
	ros2 topic echo /imu/data_raw

### View Euler angles
	ros2 topic echo /imu/euler_angles

## Test the TCP connection

#### Connect to the WebSocket bridge
	telnet localhost 8765

#### You should receive JSON data every 50 ms:
	 {"pitch": -1.23, "roll": 0.45, "speed": 4.0, "battery_voltage": 11.8, "rudder_angle": 1.0, "yaw": 180.0}

Configuration

    Sampling rate
    Edit imu_publisher.py:

	timer_period = 0.02  # 50 Hz (default)

	WebSocket port

Edit websocket_bridge.py:

	self.port = 8765  # change if desired

IMU filtering parameters
In mini_imu.py, adjust:

    alpha = 0.95  # complementary filter (0.0–1.0)
    # closer to 1.0 = more gyroscope influence
    # closer to 0.0 = more accelerometer influence

---
# Flutter App Integration

#### This package works with a Flutter app. 
See:  https://github.com/PawelG1/ROSBoatControlPanel.git

#### Telemetry data is sent as JSON over TCP on port 8765.

```
{
  "pitch": -1.23,
  "roll": 0.45,
  "speed": 4.0,
  "battery_voltage": 11.8,
  "rudder_angle": 1.0,
  "yaw": 180.0
}
```
# Troubleshooting
## IMU not detected

 ### Check I2C connection
```
i2cdetect -y 1
```
#### You should see:
 0x1e – LIS3MDL magnetometer
 0x6b – LSM6DS33 accelerometer/gyroscope

## I2C permission errors

 Add your user to the i2c group
	
	sudo usermod -a -G i2c $USER
 
Then log out and log back in

## Unstable readings

    Ensure calibration was completed

    Check I2C wiring quality

    Tune filtering parameters in mini_imu.py

## WebSocket not working

 Check if the port is in use
	
	netstat -tulpn | grep :8765

 Run the node with debug logs

	ros2 run submarine_pkg websocket_bridge --ros-args --log-level debug




# Note: This package is a work in progress (WIP). Use at your own risk 
