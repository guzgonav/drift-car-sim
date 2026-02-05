"""
Launch file for drift car simulation with ROS2 bridge

Launches:
    1. Gazebo Sim with selected world (track or empty)
    2. ros_gz_bridge for topic bridging (ROS2 <-> Gazebo)

Usage:
    ros2 launch drift_car_bringup simulation.launch.py              # Default: track
    ros2 launch drift_car_bringup simulation.launch.py world:=track # Circular track
    ros2 launch drift_car_bringup simulation.launch.py world:=empty # Empty world (faster)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Deferred setup to resolve LaunchConfiguration values."""

    # Get package directories
    pkg_drift_car = get_package_share_directory('drift_car_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resolve the world argument
    world_arg = LaunchConfiguration('world').perform(context)

    # Map world argument to file
    world_files = {
        'track': 'drift_track.sdf',
        'empty': 'empty_world.sdf'
    }
    world_filename = world_files.get(world_arg, 'drift_track.sdf')
    world_file = os.path.join(pkg_drift_car, 'worlds', world_filename)

    bridge_config = os.path.join(pkg_drift_car, 'config', 'bridge_config.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    return [gazebo, bridge]


def generate_launch_description():

    # Get package directories
    pkg_drift_car = get_package_share_directory('drift_car_bringup')
    models_path = os.path.join(pkg_drift_car, 'models')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find our models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )
    
    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'world',
            default_value='track',
            description='World to load: "track" (circular drift track) or "empty" (flat ground, faster)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_setup),
    ])
