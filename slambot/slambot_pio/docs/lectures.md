# Lectures

[] **a_introduction_to_esp32_and_microcontroller_fundamentals :**
- Overview of ESP32 architecture and capabilities
- Differences between microcontrollers and microprocessors
- Power requirements, GPIO pins, and onboard peripherals
- Typical use cases for IoT and robotics applications
- Setting the stage for robot development using ESP32

---

[] **b_setting_up_the_platformio_development_environment :**
- Installation of Visual Studio Code and PlatformIO extension
- Creating and configuring a new PlatformIO project for ESP32
- Introducing folder structure and build/upload processes
- Basic troubleshooting of toolchain errors
- Benefits of PlatformIO over classic Arduino IDE

---

[] **c_robot_building_basic** *(physical video required)*

---

[] **d_Electronics_Explanation_Wiring_Diagrams_also Cnnecting Robot** *(Physical video required)*
- Tell only electronics and power requirments
- Donot tell how they work-> it will be tld later

---


[] **ea_how mtoor drives_ pwm , 256 bits , connections :**
[x] **eb_code_motor_test :**
[] **ec_car_serial_driving_output :**

- Write test motors code
- Creating a dedicated `MotorControl` class for clarity
- Initializing motor control pins within the constructor
- Explaining PWM signals for speed control
- Testing forward, backward, turn, and stop methods
- Printing debug messages to ensure correct motor responses

---

[] **fa_how_encoders_work_isr :**
[] **fb_code_test_encoders :**
[] **fc_encoder_physical_values_testing :**

- Wiring details for left and right encoder signals
- Implementing the `EncoderSensor` class
- Polling encoder counts for speed/distance measurements
- Using interrupts vs. polling for more accurate data
- Validating counts with manual wheel rotations

---
[] **ga_how_mpy_works**
[] **gb_code_test_mpu**
[] **gc_mpu_values_check**
- I2C wiring and addressing (SDA and SCL lines)
- Initializing the MPU9250 library and calibration basics
- Reading accelerometer and gyro values
- Understanding sensor units (m/s², rad/s)
- Verifying stable readings on a flat surface

---

[] **ha_gather_all_libraries->WHY**
[] **hb_code_creating_libraries**
[] **hc_testing_live**



---

[] **i_howMicroROS_Works_DDS_etc**

[] **ja_micro_ros_setup_on_the_esp32_communcicateion**
[] **jb_how_communication_from_pc_works**
[] **jc_communication_testing**

---

[] **k_our_robot_to_microROS_plan**
- explain connection - topics , nodes - allocators what doing
- Explain cmd_vel to motion system


---

[] **l_writing_microROS_interface_for_our_robot**
- Linking the `geometry_msgs/msg/Twist` subscriber
- Writing the callback function to control motors
- Mapping linear and angular velocity to PWM signals
- Demonstrating how velocity commands drive the robot
- Validating with ROS 2 teleoperation tools

---

[] **m_testing_the_complete_robot_with_micro_ros :** *(Physical video required)*
- Recording the fully assembled robot responding to commands
- Showing live sensor streams (IMU, encoders) on a ROS 2 terminal
- Observing real-time velocity commands from a PC
- Checking alignment and any drift corrections
- Celebrating a fully ROS-ready robot on ESP32

---

[] **n_setting_up_raspberry_pi_4_with_ubuntu_server_22_04 :**
- Downloading and flashing the Ubuntu image to an SD card
- Configuring headless operation (SSH/serial console)
- Enabling Wi-Fi by editing network configuration files
- Basic system updates and security setups
- Verifying remote access to Raspberry Pi

---

[] **o_installing_ros2_on_raspberry_pi :**
- Selecting the correct ROS 2 distribution for Ubuntu 22.04
- Adding ROS 2 apt repositories and running installations
- Sourcing ROS 2 setup scripts for every new shell
- Testing the installation with a sample talker/listener
- Ensuring Pi has enough resources and swap if needed

---

[] **p_connecting_esp32_and_2d_lidar_to_raspberry_pi :** *(Physical video required)*
- Demonstrating hardware connections (USB for ESP32, USB/serial for LIDAR)
- Installing necessary LIDAR drivers or ROS 2 packages
- Confirming that the Pi recognizes both devices
- Running LIDAR node and echoing scan topics
- Verifying combined data flow (robot + LIDAR) on the Pi

---

[] **q_full_system_test_ros_based_mobile_robot_with_lidar :** *(Physical video required)*
- Launching the micro-ROS agent on the Raspberry Pi
- Streaming sensor and LIDAR data into a single ROS 2 environment
- Teleoperating the robot while visualizing LIDAR scans
- Observing real-time feedback in RViz or a similar tool
- Wrapping up all steps into a functional, ROS-enabled robot system