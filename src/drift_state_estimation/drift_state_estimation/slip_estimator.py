#!/usr/bin/env python3
"""
Slip Estimator Node

Combines IMU, odometry, and wheel data to estimate the drift state.
Calculates slip angle (β) - the key metric for drift control.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from drift_msgs.msg import ImuProcessed, WheelSpeeds, DriftState
from drift_state_estimation.utils.transforms import normalize_angle
from drift_state_estimation.utils.filters import low_pass_filter


class SlipEstimator(Node):
    def __init__(self):
        super().__init__('slip_estimator')

        # Declare parameters
        self.declare_parameter('drift_threshold', 0.087)  # ~5 degrees in radians
        self.declare_parameter('slip_angle_filter', 0.2)

        # Get parameters
        self.drift_threshold = self.get_parameter('drift_threshold').value
        self.slip_angle_filter = self.get_parameter('slip_angle_filter').value

        # Publisher
        self.pub_drift_state = self.create_publisher(
            DriftState,
            '/drift/state',
            10
        )

        # Subscribers
        self.sub_imu = self.create_subscription(
            ImuProcessed,
            '/drift/imu_processed',
            self.imu_callback,
            10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.sub_wheel_speeds = self.create_subscription(
            WheelSpeeds,
            '/drift/wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        self.sub_wheel_velocity = self.create_subscription(
            TwistStamped,
            '/drift/wheel_velocity',
            self.wheel_velocity_callback,
            10
        )

        # Stored data
        self.latest_imu = None
        self.latest_odom = None
        self.latest_wheel_speeds = None
        self.latest_wheel_velocity = None

        # Previous slip angle for filtering
        self.prev_slip_angle = None

        # Create a timer to publish drift state at a fixed rate
        self.create_timer(0.02, self.publish_drift_state)  # 50 Hz

        self.get_logger().info('Slip Estimator node started')
        self.get_logger().info(f'Drift threshold: {math.degrees(self.drift_threshold):.1f} degrees')

    def imu_callback(self, msg):
        """Store latest IMU data."""
        self.latest_imu = msg

    def odom_callback(self, msg):
        """Store latest odometry data."""
        self.latest_odom = msg

    def wheel_speeds_callback(self, msg):
        """Store latest wheel speeds."""
        self.latest_wheel_speeds = msg

    def wheel_velocity_callback(self, msg):
        """Store latest wheel velocity estimate."""
        self.latest_wheel_velocity = msg

    def publish_drift_state(self):
        """Compute and publish drift state."""
        # Check if we have all required data
        if (self.latest_imu is None or
            self.latest_odom is None or
            self.latest_wheel_speeds is None):
            return

        # Extract data from odometry
        odom = self.latest_odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        vx = odom.twist.twist.linear.x  # Forward velocity
        vy = odom.twist.twist.linear.y  # Lateral velocity

        # Extract data from IMU
        imu = self.latest_imu
        yaw = imu.yaw
        yaw_rate = imu.yaw_rate
        lateral_accel = imu.lateral_accel

        # Extract wheel speeds
        wheels = self.latest_wheel_speeds

        # Calculate velocity magnitude
        v_magnitude = math.sqrt(vx * vx + vy * vy)

        # Calculate slip angle (β)
        # Slip angle is the difference between velocity direction and heading
        if v_magnitude > 0.1:  # Only calculate if moving
            velocity_angle = math.atan2(vy, vx)
            slip_angle = normalize_angle(velocity_angle - yaw)
        else:
            slip_angle = 0.0

        # Apply low-pass filter to slip angle
        slip_angle = low_pass_filter(slip_angle, self.prev_slip_angle, self.slip_angle_filter)
        self.prev_slip_angle = slip_angle

        # Determine if drifting
        is_drifting = abs(slip_angle) > self.drift_threshold

        # Get steering angle from wheel speeds (if available)
        # For now, we'll set it to 0 - can be improved later
        steering_angle = 0.0

        # Create and publish DriftState message
        drift_state = DriftState()
        drift_state.header.stamp = self.get_clock().now().to_msg()
        drift_state.header.frame_id = 'odom'

        # Pose
        drift_state.x = x
        drift_state.y = y
        drift_state.yaw = yaw

        # Velocities
        drift_state.vx = vx
        drift_state.vy = vy
        drift_state.v_magnitude = v_magnitude
        drift_state.yaw_rate = yaw_rate

        # Drift metrics
        drift_state.slip_angle = slip_angle
        drift_state.lateral_accel = lateral_accel

        # Wheel speeds
        drift_state.wheel_speed_fl = wheels.front_left
        drift_state.wheel_speed_fr = wheels.front_right
        drift_state.wheel_speed_rl = wheels.rear_left
        drift_state.wheel_speed_rr = wheels.rear_right

        # Steering
        drift_state.steering_angle = steering_angle

        # Status
        drift_state.is_drifting = is_drifting

        self.pub_drift_state.publish(drift_state)

        # Log when drift is detected (throttled)
        if is_drifting:
            self.get_logger().info(
                f'DRIFTING! Slip angle: {math.degrees(slip_angle):.1f}°',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlipEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
