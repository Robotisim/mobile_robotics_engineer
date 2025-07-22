cmake_minimum_required(VERSION 3.8)
project(slambot_bringup)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable for odometry generation
add_executable(odom_gen_node src/odom_generator.cpp)

# Link dependencies to the executable
ament_target_dependencies(odom_gen_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(imu_fix src/imu_fix.cpp)

ament_target_dependencies(imu_fix
rclcpp
tf2
  tf2_ros
  sensor_msgs
)

# Install the executable
install(TARGETS
  odom_gen_node imu_fix
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}/
)




if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
