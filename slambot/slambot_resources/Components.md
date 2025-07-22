# Components List for SLAMBot
### 🛠️ **1. Mechanical Structure & Assembly**
These parts are needed to physically build the body of the robot.

- **3D Printing Parts**
  Custom-designed parts for the robot chassis, motor mounts, sensor holders, etc.
- **Caster Wheel (Self Adhesive Mini Swivel Casters)**
  A small omnidirectional wheel for balance—usually placed at the rear or front. [See here](images/caster_wheel.png)
- **M3 Screws, Nuts, and Bolts x 10**
  For assembling 3D-printed parts and securing motors, caster, and board mounts.

---

### ⚙️ **2. Mobility: Motors & Drivers**
Components required for motion control of the robot.

- **2x Encoder Motors with Wheel ( JGA25-370 12V 280RPM )**
  Motors that provide feedback on speed and rotation using encoders.[See here](images/motors.png)
- **2x Wheels ( 70mm diameter ) with couplings**
- **L298D Motor Driver**
  A dual H-bridge driver to control the speed and direction of motors.
- **Jumper Wires (10x female-to-female)**
  For connecting modules (like motor driver to ESP32) easily.

---

### 🔋 **3. Power Supply & Distribution**
Powering the motors, controller, and sensors.

- **3x 18650 Cells + 3S Holder**
  Rechargeable Li-ion batteries to provide main power.
- **Vero Board (in series)**
  A prototyping board for distributing power and organizing connections.
- **2.54mm Pitch 40 Pin Single Row Female Header**
  For connecting the Vero board to the L298D and ESP32.
- **2.54mm Pitch Male Headers 40 pin**

- **Power Button ( rocker switch mini 6A)**
  A toggle to safely turn the robot on/off.
- **PowerBank (xiomi power bank pro 10000mah)**
  Separate power supply for Raspberry Pi 4 to avoid voltage drops.[See here](images/power_bank.png)
- **Simple Wires, Cutter, Solder, Solder Wire**
  Essential tools for making permanent and stable electrical connections.
- **18650 charger**
  A charger for the 18650 cells to keep them powered.

---

### 🧠 **4. Control & Communication**
Processing units that run the robot’s code and logic.

- **ESP32 Dev Kit V1**
  Main microcontroller to control motors, read sensors, and handle logic.
- **Raspberry Pi 4 ( 4gb )**
  A powerful onboard computer used for ROS, SLAM, or higher-level decision making.
- **2x Type A USB Cable,1x Type C USB Cable**
  For programming and powering both ESP32 and Raspberry Pi.

---

### 🧭 **5. Sensors & Perception**
To sense the environment and robot’s own movement.

- **MPU6050 (IMU Sensor)**
  Provides gyroscope and accelerometer data to estimate orientation and motion.
- **2D RP-LiDAR**
  Used for mapping, obstacle detection, and autonomous navigation.
