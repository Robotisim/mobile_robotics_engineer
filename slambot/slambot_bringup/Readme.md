### Communicating with Robot
- SSH RPI
ssh pi@<RPI_IP_ADDRESS>
- Start Docker for MicroROS Agent
```
docker run -it --rm \
  --privileged \
  --net=host \
  -v /dev:/dev \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0

```
- Lidar Scanning Start
```
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

----
### Recording things
- IMU from esp32 needs timestamp + frameid -> write node imu_fix
- Understand linear and angular velocity from IMU , By rotation of robot , Interconnected things and opposite things connected
- To get Orientation we need to apply amdgwick filter and test Z axis orientation
```
sudo apt install ros-humble-imu-filter-madgwick
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -r imu/data_raw:=/imu/data_fixed -r imu/data:=/imu/data_filtered
```
- Fix data publishing according to REP-103 , incorrect setting , incorrect output
- Aply test_caliberation system in test direction
- Then apply those offset into main micro ROS script
- Visualize the data in Rviz2 with madghich filer  - it will give clearn output.

---
### Terminal commands for Robot localization -> IMU Problem Solving
- SSH PI
```
ssh pi@192.168.100.18
```
- Docker in PI MicroROS
```
docker run -it --rm   --privileged   --net=host   -v /dev:/dev   microros/micro-ros-agent:humble   serial --dev /dev/ttyUSB0
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```
- Rviz2
- Odom generation
```
ros2 run slambot_bringup odom_gen_node
```
- IMU orientation calculation
```
ros2 run slambot_bringup imu_fix
```
- Madwigwick ilter apply
```
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -r imu/data_raw:=/imu/data_fixed -r imu/data:=/imu/data_filtered
```
- Robot Loclaization Running
```
ros2 run robot_localization ekf_node --ros-args --params-file /home/luqman/robotisim_ws/src/dev_robotisim/slambot/slambot_bringup/config/ekf.yaml
```
- Trasnform for IMu to base link
```
 ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
```
- teleop Drive Robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Saving Map
```
ros2 run nav2_map_server map_saver_cli -f my_saved_map
```

###
----

