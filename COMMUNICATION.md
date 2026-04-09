# RPi ↔ Flutter App – Communication Architecture

This document describes, step by step, how data flows from the sensors on the
Raspberry Pi all the way to the Flutter app displayed on a phone or tablet.
It is written so that it can be presented to a supervisor without any
prior ROS2 knowledge.

---

## Overview

```
┌─────────────────────────────────────────────────────┐
│               Raspberry Pi 5 (Ubuntu)               │
│                                                     │
│  I2C sensor ──► imu_publisher ──► /imu/euler_angles │
│  (LSM6DS33)       (ROS2 node)      (ROS2 topic)     │
│                                         │           │
│                                         ▼           │
│  USB camera ──► gscam/v4l2 ──► /camera/image_raw   │
│                  (ROS2 node)     (ROS2 topic)        │
│                                         │           │
│              websocket_bridge           │           │
│              (ROS2 node)       ◄────────┘           │
│                 │                                   │
│                 │ WebSocket (port 8765)              │
│                 │ HTTP MJPEG (port 8080)             │
└─────────────────┼───────────────────────────────────┘
                  │  Direct Ethernet cable
              10.0.0.2 ──────────────── 10.0.0.1
                                    (laptop / phone)
                                         │
                                   Flutter app
                                (ROSBoatControlPanel)
```

---

## 1. Physical Network Layer

The Raspberry Pi and the device running the Flutter app are connected through
a **direct Ethernet cable** (no router, no Wi-Fi).

| Device          | Static IP address |
|-----------------|-------------------|
| Raspberry Pi 5  | `10.0.0.2`        |
| Laptop / tablet | `10.0.0.1`        |

Using a direct cable gives the lowest possible latency (sub-millisecond round
trip) and avoids any dependency on an external network.

---

## 2. ROS2 Layer – IMU data pipeline

ROS2 is the middleware that connects all software components on the Raspberry
Pi via a publish/subscribe message bus.

### 2a. `imu_publisher` node (`imu_publisher.py`)

1. **Reads the IMU sensor** (LSM6DS33 accelerometer + gyroscope) over the I2C
   bus at **50 Hz** (every 20 ms).
2. **Corrects** the raw readings with pre-computed calibration biases stored in
   `calibration.json`.
3. **Filters** the corrected data with a *complementary filter* (combines
   accelerometer and gyroscope to eliminate drift and noise) to produce stable
   Euler angles: **pitch**, **roll**, and **yaw** in degrees.
4. **Publishes** two ROS2 topics:
   - `/imu/data_raw` – raw accelerometer + gyroscope values
     (`sensor_msgs/Imu`, 50 Hz)
   - `/imu/euler_angles` – filtered pitch / roll / yaw angles
     (`geometry_msgs/Vector3`, 50 Hz)

### 2b. `websocket_bridge` node (`websocket_bridge.py`)

1. **Subscribes** to `/imu/euler_angles` and caches the latest angles.
2. **Runs an async WebSocket server** on `0.0.0.0:8765` in a background thread.
3. Every **50 ms (20 Hz)** broadcasts a JSON message to every connected client:

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

| Field             | Source              | Unit    |
|-------------------|---------------------|---------|
| `pitch`           | IMU (filtered)      | degrees |
| `roll`            | IMU (filtered)      | degrees |
| `yaw`             | IMU (filtered)      | degrees |
| `speed`           | placeholder value   | –       |
| `battery_voltage` | placeholder value   | V       |
| `rudder_angle`    | placeholder value   | degrees |

---

## 3. Video Stream

The camera image follows a separate path:

1. **`gscam` (or `v4l2_camera`) node** captures frames from the USB camera and
   publishes them on the `/camera/image_raw` ROS2 topic.
2. **`web_video_server` node** subscribes to that topic and re-encodes the
   stream as **Motion JPEG (MJPEG)** served over HTTP on port **8080**.
3. The Flutter app opens an HTTP connection to:

```
http://10.0.0.2:8080/stream?topic=/camera/image_raw&type=mjpeg
```

MJPEG is used because it is a simple, widely-supported format that requires no
special decoding library in Flutter.

---

## 4. Flutter App

The Flutter app (repository: https://github.com/PawelG1/ROSBoatControlPanel)
connects to the Raspberry Pi using two simultaneous connections:

| Connection         | Protocol  | Address / URL                                              | Purpose               |
|--------------------|-----------|------------------------------------------------------------|-----------------------|
| Telemetry channel  | WebSocket | `ws://10.0.0.2:8765`                                      | IMU + boat data (JSON) |
| Video channel      | HTTP      | `http://10.0.0.2:8080/stream?topic=/camera/image_raw&type=mjpeg` | Live camera feed |

### Telemetry flow (WebSocket)

```
RPi sensor (50 Hz)
    └─► imu_publisher publishes /imu/euler_angles
            └─► websocket_bridge caches latest values
                    └─► every 50 ms builds JSON and sends to all clients
                                └─► Flutter receives JSON, parses it,
                                    updates UI widgets
```

### Video flow (HTTP MJPEG)

```
USB camera
    └─► gscam publishes /camera/image_raw (ROS2 topic)
            └─► web_video_server re-encodes as MJPEG over HTTP
                    └─► Flutter displays as a continuously refreshing image
```

---

## 5. Summary

The communication stack has three layers:

1. **Physical**: direct Ethernet cable with static IPs – no external
   infrastructure required.
2. **Middleware (ROS2)**: topics decouple the sensor driver from the network
   bridge; each component can be developed and tested independently.
3. **Application protocols**:
   - **WebSocket** for real-time bidirectional telemetry (low overhead,
     streaming-friendly).
   - **HTTP MJPEG** for video (simple, compatible with Flutter's `Image`
     widget).

All nodes are started with a single command:

```bash
ros2 launch submarine_pkg submarine_launch.py
```
