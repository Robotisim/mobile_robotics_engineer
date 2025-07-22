cmake_minimum_required(VERSION 3.8)
project(slambot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
# We don't need to find_package for libserial if it's in a standard system path

# Define the C++ hardware interface as a shared library
add_library(diffbot_system_hardware SHARED
  src/diffbot_system.cpp
)

# Tell the compiler where to find the header files for this library
target_include_directories(diffbot_system_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link the library against its ROS dependencies
ament_target_dependencies(diffbot_system_hardware PUBLIC
    "hardware_interface"
    "pluginlib"
    "rclcpp"
    "ros2_control"
)

# --- THE FIX IS HERE ---
# Link against the 'serial' library. CMake will correctly find 'libserial.so'.
target_link_libraries(diffbot_system_hardware PRIVATE
  serial
)

# Install the compiled library to the lib directory
install(TARGETS
  diffbot_system_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the plugin description file for ros2_control to find it
pluginlib_export_plugin_description_file(hardware_interface diffbot_system_hardware.xml)

# Install the launch, urdf, and config directories
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the plugin xml file
install(FILES
  diffbot_system_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()