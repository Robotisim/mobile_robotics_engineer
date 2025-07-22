from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('slambot_bringup')
    rviz_config = os.path.join(pkg_share, 'config', 'bringup.rviz')

    return LaunchDescription([
        # ---------------- Odometry node ----------------
        Node(
            package='slambot_bringup',
            executable='odom_gen_node',
            name='odom_node',
            output='screen'
        ),

        # ---------------- Static transform: base_link → laser ----------------
        Node(

            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '1.57', '0', '0', 'base_link', 'laser']

        ),

        # ---------------- SLAM Toolbox (online / mapping) ----------------
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'map_frame': 'map',
                'mode': 'mapping'
            }]
        ),

        # ---------------- RViz 2 (pre‑loaded config) ----------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        )
    ])
