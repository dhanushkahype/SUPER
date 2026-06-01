"""
PX4-specific SUPER planner launch file.
Launches SUPER fsm_node + RViz only (no perfect_drone_sim).
Expects PX4 SITL + mavros + OMMPC controller running separately.

Topics expected:
  /cloud_registered    — PointCloud2 from GZ bridge
  /lidar_slam/odom     — nav_msgs/Odometry relayed from mavros
  
Topics published:
  /planning_cmd/poly_traj — PolynomialTrajectory consumed by OMMPC
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Use SUPER's RViz config as base
    sim_pkg_path = get_package_share_directory('perfect_drone_sim')
    default_rviz_config = os.path.join(sim_pkg_path, 'rviz2', 'fpv.rviz')

    # PX4-specific SUPER config
    super_config_name = 'px4_dense.yaml'

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    ))

    # --- Static TF: world → map (identity) ---
    # SUPER uses "world" frame; mavros uses "map" frame.
    # This identity transform keeps RViz and TF tree consistent.
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='log'
    )
    ld.add_action(world_to_map_tf)

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config],
        output='screen'
    )
    ld.add_action(rviz_node)

    # --- SUPER planner (fsm_node) ---
    # Uses px4_dense.yaml which has adjusted map size, robot radius,
    # and velocity limits for PX4 SITL testing.
    super_node = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            'config_name': super_config_name,
        }]
    )
    ld.add_action(super_node)

    return ld
