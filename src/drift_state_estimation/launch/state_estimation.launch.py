"""
Launch file for drift state estimation nodes.

Launches:
- imu_processor: Processes raw IMU data
- wheel_odometry: Calculates wheel speeds and expected velocity
- slip_estimator: Estimates slip angle and drift state
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('drift_state_estimation'),
        'config',
        'estimation_params.yaml'
    )

    return LaunchDescription([
        # IMU Processor Node
        Node(
            package='drift_state_estimation',
            executable='imu_processor.py',
            name='imu_processor',
            parameters=[config_file],
            output='screen'
        ),

        # Wheel Odometry Node
        Node(
            package='drift_state_estimation',
            executable='wheel_odometry.py',
            name='wheel_odometry',
            parameters=[config_file],
            output='screen'
        ),

        # Slip Estimator Node
        Node(
            package='drift_state_estimation',
            executable='slip_estimator.py',
            name='slip_estimator',
            parameters=[config_file],
            output='screen'
        ),
    ])
