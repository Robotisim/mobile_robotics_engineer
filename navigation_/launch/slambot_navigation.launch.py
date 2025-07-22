#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slambot_pkg_dir = os.path.join(get_package_share_directory('slambot_bringup'))
    mapping_pkg_dir = os.path.join(get_package_share_directory('mapping'))
    navigation_pkg_dir=get_package_share_directory('navigation_')
    map_file = os.path.join(mapping_pkg_dir,'map','my_saved_map.yaml')
    params_file = os.path.join(navigation_pkg_dir,'config','nav2_slambot_params.yaml')
    rviz_config_file = os.path.join(navigation_pkg_dir,'config','nav2.rviz')
 # ---------------- Odometry node ----------------
    odom=Node(
            package='slambot_bringup',
            executable='odom_gen_node',
            name='odom_node',
            output='screen'
        )

        # ---------------- Static transform: base_link → laser ----------------
    laser_tf=Node(

            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '1.57', '0', '0', 'base_link', 'laser']

        )
    env_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( slambot_pkg_dir, 'launch', 'slam_bringup.launch.py')
        ),
    )


    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py']),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file
    }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", rviz_config_file],
        name='rviz'
    )

    ld = LaunchDescription()

    # ld.add_action(env_bringup)
    ld.add_action(navigation)
    ld.add_action(rviz)
    ld.add_action(laser_tf)
    ld.add_action(odom)

    return ld