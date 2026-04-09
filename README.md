# Submarine Control Package

A ROS2 package for controlling a surface/submersible boat on a Raspberry Pi, with support for the Pololu MinIMU-9 v5 IMU sensor.

## Description

This package provides a complete control system for a surface/submersible boat, enabling:

- Reading orientation data from the IMU sensor (pitch, roll, yaw)
- Transmitting telemetry data over WebSocket/TCP
- Integration with a Flutter app (in a separate repository)
- Advanced IMU data filtering with drift compensation

## Components

### IMU Sensor 3-axis accelerometer + gyroscope data
- LIS3MDL publishes IMU data
  - Topic: `/imu/data_raw` (sensor_msgs/Imu)
  - Topic: `/imu/euler_angles` (geometry_msgs/Vector3)
  - Rate: 50 Hz

2. **websocket_bridge** Jazzy)

> **Note**: ROS2 Humble is for Ubuntu 22.04. On Ubuntu 24.04 use **Jazzy**.

```bash
# Add ROS2 repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
 -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
 | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop
pip install websockets --break-system-packages

# Install additional ROS2 packages
sudo apt install ros-jazzy-gscam
sudo apt install ros-jazzy-web-video-server
```

---

### 3. Configure Direct Ethernet Connection (Raspberry Pi safe for most home networks.

#### Raspberry Pi 5 (Ubuntu)

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

> **Note**: Do not add `renderer: networkd` not an error.
```

Verify:

```bash
ip addr show eth0
# Expected: inet 10.0.0.2/24
```

#### Laptop use yours below
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

# Reconnect automatically after reboot
sudo nmcli connection modify rpi-link connection.autoconnect yes
```

#### Laptop Network & Internet Edit**.
2. Set **IP assignment** to **Manual**.
3. Enter:
  - IP address: `10.0.0.1`
  - Subnet mask: `255.255.255.0`
  - Gateway: *(leave empty)*
4. Click **Save**.

#### SSH Setup (first time only)

On the Raspberry Pi:

```bash
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

#### Verify the Connection

From the laptop:

```bash
ping 10.0.0.2
ssh rpi5@10.0.0.2
```

From the Raspberry Pi:

```bash
ping 10.0.0.1
```

#### Troubleshooting

| Symptom | Fix |
|---|---|
| `ping` fails | Run `ip addr show eth0` on Pi and `ip addr show enp2s0` on laptop verify with `ip addr show eth0` after reboot |
| Settings lost after reboot (laptop) | Run: `sudo nmcli connection modify rpi-link connection.autoconnect yes` |

---

### 4. Install the Package

```bash
# Navigate to your ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/PawelG1/submarine_ros2_pkg.git submarine_pkg

# Build the package
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select submarine_pkg

# Source the workspace
source install/setup.bash
```

---

### 5. Calibrate the IMU (IMPORTANT)

Before first use, calibrate the sensor. Keep the sensor completely still during calibration.

```bash
cd ~/ros2_ws/src/submarine_pkg/submarine_pkg
python3 mini_imu.py
```

Calibration takes ~40 seconds. A file named `calibration.json` is created automatically.

After calibration, copy it to the installed package location:

```bash
cp calibration.json ~/ros2_ws/install/submarine_pkg/lib/python3.12/site-packages/submarine_pkg/
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select submarine_pkg
source install/setup.bash
```

---

## Running

### Launch the entire system (recommended)

```bash
ros2 launch submarine_pkg submarine_launch.py
```

### Launch individual nodes

```bash
# Terminal 1 WebSocket Bridge
ros2 run submarine_pkg websocket_bridge
```

> If launch fails with `Address already in use` on port 8765, kill the leftover process first:
> ```bash
> sudo fuser -k 8765/tcp
> ```

---

## Monitoring Data

```bash
# List available topics
ros2 topic list

# View raw IMU data
ros2 topic echo /imu/data_raw

# View Euler angles
ros2 topic echo /imu/euler_angles
```

### Test the TCP connection

```bash
telnet localhost 8765
```

You should receive JSON data every 50 ms:

```json
{"pitch": -1.23, "roll": 0.45, "speed": 4.0, "battery_voltage": 11.8, "rudder_angle": 1.0, "yaw": 180.0}
```

---

## Configuration

**Sampling rate** edit `websocket_bridge.py`:

```python
self.port = 8765  # change if desired
```

**IMU filtering parameters** 1.0)
# closer to 1.0 = more gyroscope influence
# closer to 0.0 = more accelerometer influence
```

---

## Flutter App Integration

This package works with a Flutter app.
See: https://github.com/PawelG1/ROSBoatControlPanel.git

For a detailed, step-by-step explanation of the full communication architecture
(network layer, ROS2 topics, WebSocket bridge, JSON format, video stream) see
**[COMMUNICATION.md](COMMUNICATION.md)**.

Telemetry data is sent as JSON over TCP on port 8765:

```json
{
 "pitch": -1.23,
 "roll": 0.45,
 "speed": 4.0,
 "battery_voltage": 11.8,
 "rudder_angle": 1.0,
 "yaw": 180.0
}
```

Video stream is available via HTTP on port 8080:

```
http://10.0.0.2:8080/stream?topic=/camera/image_raw&type=mjpeg
```

---

## Troubleshooting

### IMU not detected

```bash
sudo i2cdetect -y 1
```

Expected addresses:
- `0x1e` LSM6DS33 accelerometer/gyroscope

### I2C permission errors

```bash
sudo usermod -a -G i2c $USER
# Log out and log back in
```

### Calibration file not found

```bash
cp ~/ros2_ws/src/submarine_pkg/submarine_pkg/calibration.json \
  ~/ros2_ws/install/submarine_pkg/lib/python3.12/site-packages/submarine_pkg/
```

### Unstable readings

- Ensure calibration was completed with sensor held still
- Check I2C wiring quality
- Tune filtering parameters in `mini_imu.py`

### WebSocket not working

```bash
# Check if the port is in use
netstat -tulpn | grep :8765

# Kill leftover process
sudo fuser -k 8765/tcp

# Run node with debug logs
ros2 run submarine_pkg websocket_bridge --ros-args --log-level debug
```

---

> **Note**: This package is a work in progress (WIP). Use at your own risk.
