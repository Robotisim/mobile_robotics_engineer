# 2D SLAM Robot

This project builds a 2D SLAM robot with split responsibilities:

- The **ESP32‑S3** acts as a **low‑level controller**, handling DC motor drive and encoder feedback, IMU data for inertial sensing.
- A **Raspberry Pi 4** serves as the **high‑level controller**, performing SLAM algorithms based on LiDAR and camera inputs. The Pi computes the robot’s position/map and sends motion commands back to the ESP32‑S3.

Communication between the ESP32‑S3 and Raspberry Pi can be either **Serial (UART)**, using [micro-ROS](https://micro.ros.org/) to integrate with the ROS 2 ecosystem.

---

## ESP32‑S3 Pin Assignments & Wiring

Below is an **example** wiring layout. Adapt if your setup differs.

```
          +-------------------------------------+
          |         ESP32‑S3 DevKit             |
          |                                     |
          |  Motor Driver – Left Motor          |
          |    GPIO19  -->  ENA (PWM)           |
          |    GPIO20  -->  IN1 (Dir)           |
          |    GPIO21  -->  IN2 (Dir)           |
          |                                     |
          |  Motor Driver – Right Motor         |
          |    GPIO47  -->  ENB (PWM)           |
          |    GPIO48  -->  IN3 (Dir)           |
          |    GPIO45* -->  IN4 (Dir)           |
          |                                     |
          |  Encoder Connections:               |
          |   Encoder 1 (Left Motor):           |
          |     Channel A: GPIO6                |
          |     Channel B: GPIO4                |
          |   Encoder 2 (Right Motor):          |
          |     Channel A: GPIO5                |
          |     Channel B: GPIO7                |
          |                                     |
          |  I²C IMU Sensor                     |
          |     SDA: GPIO41                     |
          |     SCL: GPIO42                     |
          +-------------------------------------+
```


---
### IMU Placement
```
+Z  → Up
+X  → Back of robot
+Y  → Right of robot
```

## Using micro-ROS for Communication

### On the Raspberry Pi: micro-ROS Agent : ONCE

Using micro-ROS over **serial**, follow these **one-time setup** steps on the Pi :

```bash
# Create and enter a workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone micro_ros_setup for your ROS 2 distro (e.g., humble)
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y python3-pip

# Build and source
colcon build
source install/local_setup.bash

# Create and build the Agent workspace
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

**To run** the agent each time you want to connect microController
```bash
cd ~/microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
---

## Typical Workflow

1. **Power on the robot** (ESP32-S3, motors, sensors).
2. **Run** the micro-ROS Agent on the Raspberry Pi:
   ```bash
   cd ~/microros_ws
   source install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```
3. **Start** your SLAM software on the Pi (e.g., `ros2 launch ...`).
4. The ESP32‑S3 micro-ROS node publishes odometry and IMU data, receives velocity commands, and drives motors accordingly.



## Lidar Setup
- Clone and build
```
git clone https://github.com/Slamtec/sllidar_ros2.git
```
- To visualize with `laser` frame as fixed
```
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

## Software Overview

1. **Motor Control (on ESP32‑S3)**
   - Generates PWM signals to the L298D driver for speed control.
   - Simple functions to move **forward**, **backward**, **turn left**, and **turn right**.

2. **Encoder Reading (on ESP32‑S3)**
   - Uses interrupt or peripheral-based quadrature decoding for odometry.
   - Provides real-time wheel rotation counts.

3. **Communication Module**
   - **ESP32‑S3** ↔ **Raspberry Pi** via **Serial UART** or **Wi‑Fi**.
   - On the Pi, a **micro-ROS Agent** can translate these messages into standard ROS 2 topics.
   - The Pi sends velocity commands (e.g., `/cmd_vel`); the ESP32-S3 returns odometry data, encoder counts, IMU data, etc.

4. **High-Level SLAM (on Raspberry Pi 4)**
   - Processes LiDAR (e.g., RPLIDAR, Hokuyo) and/or camera data.
   - Runs a SLAM stack (e.g., `gmapping`, `cartographer`, or `hector_slam`).
   - Publishes navigation goals and real-time velocity commands to the robot’s base controller (ESP32‑S3).

---

## Hardware Components

- **ESP32‑S3 DevKit**
  Low-level microcontroller running motor control, encoder reading, IMU sampling, and micro-ROS.

- **L298D Motor Driver**
  Drives two DC motors with PWM control signals from the ESP32-S3.

- **2 DC Motors**
  Provide forward/reverse movement and turning.

- **2 Quadrature Encoders**
  Attached to the motor shafts, giving odometry feedback (counts per revolution).

- **I²C IMU Sensor**
  E.g., MPU9250 or similar, for inertial measurements.

- **Raspberry Pi 4**
  Runs LiDAR and camera drivers, handles SLAM algorithms, and communicates with the ESP32-S3 for motor commands.

---
