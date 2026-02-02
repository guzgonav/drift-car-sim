"""
Launch file for drift car simulation with ROS2 bridge

Launches: 
    1. Gazebo Sim with drift_track world
    2. ros_gz_bridge for topic bridging (ROS2 <-> Gazebo)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directories
    pkg_drift_car = get_package_share_directory('drift_car_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_drift_car, 'worlds', 'drift_track.sdf')
    bridge_config = os.path.join(pkg_drift_car, 'config', 'bridge_config.yaml')
    models_path = os.path.join(pkg_drift_car, 'models')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find our models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo,
        bridge,
    ])
