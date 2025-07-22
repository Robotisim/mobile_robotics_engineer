### RPI Robot
- docker run -it --rm   --privileged   --net=host   -v /dev:/dev   microros/micro-ros-agent:humble   serial --dev /dev/ttyUSB0
- ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
- Test mapping first
    - ros2 launch  slambot_bringup slam_bringup.launch.py
- Test navigation
    - ros2 slambot_bringup odom_tf_ready.launch.py
    - ros2 launch navigation_ nav2_stack.launch.py

- Lecture
    - Files changes , nav2 params in navigation_ folder
    - Make them stuck on the point of map not publishing because of AMCL
    - Arrange Launch files to make sense ( slambot_bringup ,  odom publishing)

----
### Lectures
**M6_V01_Introduction_to_Nav2_**[A]
**M6_V02_Running_Complete_Nav2_Simulation_Example_**[B]
**M6_V03_Making_Changes_in_Nav2_Parameters_**[B]
**M6_V04_Integrating_Nav2_into_Our_Codebase_**[C]
**M6_V05_Understanding_Nav2_Commander_Interface_FACE_**[C]
**M6_V06_Auto_Initialization_and_Goal_Sending_in_Nav2_**[D]
[U] **M6_V07_multi_goal_sending_commander_api**[D]
**M6_V08_Testing_Different_Planner_Parameters_in_Nav2_**[D]
[U] **M6_V09_nav2_overview_Custom_algorithm**[E]
[U] **M6_V10_dwa_implementation**[E]
[U] **M6_V11_Running_nav2_on_slambot**[E]
- Error of fixed velocity - over shooting from
- Solving with PWM value oriented to cmd_vel