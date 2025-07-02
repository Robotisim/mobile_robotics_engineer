cmake_minimum_required(VERSION 3.8)
project(module_1_assignment)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies - only the essential ones for building
find_package(ament_cmake REQUIRED)

# Install directories - this tells ROS 2 where to find your files
# URDF files contain robot descriptions
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
)

# Launch files contain startup configurations
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# RViz configuration files for visualization setups
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Config files for parameters and settings
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.xml"
)

# Install Python launch files with executable permissions
install(PROGRAMS
  launch/robot_display.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing setup - ensures your package meets ROS 2 quality standards
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Run standard ROS 2 linting checks
  ament_lint_auto_find_test_dependencies()
endif()

# Register package with ament - this makes your package discoverable
ament_package()