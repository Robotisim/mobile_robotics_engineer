#!/usr/bin/env python3

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def validate_urdf_file(urdf_path: str, robot_name: str) -> bool:
    """
    Validate that the URDF file exists and is readable.
    
    Args:
        urdf_path: Path to the URDF file
        robot_name: Name of the robot for error messages
        
    Returns:
        bool: True if file is valid, False otherwise
    """
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found: {urdf_path}")
        print(f"Available robots in urdf directory:")
        urdf_dir = os.path.dirname(urdf_path)
        if os.path.exists(urdf_dir):
            urdf_files = [f for f in os.listdir(urdf_dir) if f.endswith('.urdf')]
            for file in urdf_files:
                print(f"  - {file.replace('.urdf', '')}")
        else:
            print(f"  URDF directory not found: {urdf_dir}")
        return False
    
    if not os.access(urdf_path, os.R_OK):
        print(f"ERROR: Cannot read URDF file: {urdf_path}")
        return False
    
    return True


def get_robot_description(urdf_path: str) -> str:
    """
    Read and return the robot description from URDF file.
    
    Args:
        urdf_path: Path to the URDF file
        
    Returns:
        str: Content of the URDF file
        
    Raises:
        FileNotFoundError: If URDF file cannot be read
    """
    try:
        with open(urdf_path, 'r') as urdf_file:
            return urdf_file.read()
    except Exception as e:
        raise FileNotFoundError(f"Failed to read URDF file {urdf_path}: {str(e)}")


def create_robot_state_publisher(robot_description: str, use_sim_time: bool, 
                                namespace: str = '') -> Node:
    """Create robot state publisher node."""
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': f"{namespace}/" if namespace else ""
        }],
        output='screen',
        emulate_tty=True
    )


def create_joint_state_publisher(namespace: str = '') -> Node:
    """Create joint state publisher node."""
    return Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        emulate_tty=True
    )


def create_joint_state_publisher_gui(namespace: str = '') -> Node:
    """Create joint state publisher GUI node."""
    return Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        output='screen',
        emulate_tty=True
    )


def create_rviz_node(rviz_config_path: str = '', namespace: str = '') -> Node:
    """Create RViz2 node with optional config file."""
    parameters = []
    if rviz_config_path and os.path.exists(rviz_config_path):
        parameters.extend(['-d', rviz_config_path])
    
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=parameters,
        output='screen',
        emulate_tty=True
    )


def launch_setup(context, *args, **kwargs):
    """
    Setup function to configure and launch all nodes.
    
    This function is called by the OpaqueFunction and receives the resolved
    launch configuration values.
    """
    # Get launch configuration values
    robot_name = LaunchConfiguration('robot').perform(context)
    use_gui = LaunchConfiguration('use_gui').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    
    # Print launch configuration for debugging
    print(f"\n=== Robot Visualization Launch Configuration ===")
    print(f"Robot: {robot_name}")
    print(f"Use GUI: {use_gui}")
    print(f"Use Sim Time: {use_sim_time}")
    print(f"Namespace: {namespace if namespace else 'None'}")
    print(f"RViz Config: {rviz_config if rviz_config else 'Default'}")
    print("=" * 48)
    
    # Build URDF file path
    try:
        pkg_share = get_package_share_directory('module_1_assignment')
    except Exception as e:
        print(f"ERROR: Cannot find package 'module_1_assignment': {e}")
        sys.exit(1)
    
    urdf_file = os.path.join(pkg_share, 'urdf', f'{robot_name}.urdf')
    
    # Validate URDF file
    if not validate_urdf_file(urdf_file, robot_name):
        sys.exit(1)
    
    # Read robot description
    try:
        robot_description_content = get_robot_description(urdf_file)
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    
    # Build RViz config path if specified
    rviz_config_path = ''
    if rviz_config:
        if os.path.isabs(rviz_config):
            rviz_config_path = rviz_config
        else:
            rviz_config_path = os.path.join(pkg_share, 'rviz', rviz_config)
        
        if not os.path.exists(rviz_config_path):
            print(f"WARNING: RViz config file not found: {rviz_config_path}")
            print("Using default RViz configuration")
            rviz_config_path = ''
    
    # Create nodes
    nodes = []
    
    # Robot state publisher (always needed)
    nodes.append(create_robot_state_publisher(
        robot_description_content, use_sim_time, namespace
    ))
    
    # Joint state publisher (always needed unless GUI is used)
    if not use_gui:
        nodes.append(create_joint_state_publisher(namespace))
    
    # Joint state publisher GUI (optional)
    if use_gui:
        nodes.append(create_joint_state_publisher_gui(namespace))
    
    # RViz2
    nodes.append(create_rviz_node(rviz_config_path, namespace))
    
    # Add launch info
    nodes.insert(0, LogInfo(
        msg=f"Launching robot visualization for '{robot_name}' "
            f"{'with GUI' if use_gui else 'without GUI'}"
    ))
    
    return nodes


def generate_launch_description():
    """
    Generate the launch description with all arguments and nodes.
    
    Returns:
        LaunchDescription: Complete launch description
    """
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot',
            default_value='robot_arm',
            description='Name of the robot URDF file (without .urdf extension)',
            choices=None  # Could be populated dynamically by scanning urdf directory
        ),
        
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Whether to start joint_state_publisher_gui for interactive control',
            choices=['true', 'false']
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if running with a simulator',
            choices=['true', 'false']
        ),
        
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for all nodes (useful for multi-robot scenarios)'
        ),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='RViz configuration file (relative to package rviz/ directory or absolute path)'
        )
    ]
    
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    # This allows the file to be tested directly
    print("This is a ROS2 launch file. Use with 'ros2 launch' command.")
    print("Example: ros2 launch module_1_assignment improved_launch.py robot:=my_robot use_gui:=false")