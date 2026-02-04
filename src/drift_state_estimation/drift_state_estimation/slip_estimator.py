#!/usr/bin/env python3
"""
Slip Estimator Node

Combines IMU, odometry, and wheel data to estimate the drift state.
Calculates slip angle (β) - the key metric for drift control.

Supports three methods:
1. Ground Truth (for validation) - uses actual chassis velocities
2. Integration (real-time) - integrates lateral acceleration
3. Observer/EKF (future) - model-based estimation
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from drift_msgs.msg import ImuProcessed, WheelSpeeds, DriftState
from drift_state_estimation.utils.transforms import normalize_angle, quaternion_to_euler
from drift_state_estimation.utils.filters import low_pass_filter


class SlipEstimator(Node):
    def __init__(self):
        super().__init__('slip_estimator')

        # Declare parameters
        self.declare_parameter('drift_threshold', 0.087)  # ~5 degrees in radians
        self.declare_parameter('slip_angle_filter', 0.2)
        self.declare_parameter('estimation_method', 'integration')  # 'ground_truth' or 'integration'
        self.declare_parameter('reset_integration_threshold', 0.5)  # Reset when ay and yaw_rate both low

        # Get parameters
        self.drift_threshold = self.get_parameter('drift_threshold').value
        self.slip_angle_filter = self.get_parameter('slip_angle_filter').value
        self.estimation_method = self.get_parameter('estimation_method').value
        self.reset_threshold = self.get_parameter('reset_integration_threshold').value

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
        self.sub_odom_gt = self.create_subscription(
            Odometry,
            '/odom_ground_truth',
            self.odom_gt_callback,
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
        self.latest_odom_gt = None
        self.latest_wheel_speeds = None
        self.latest_wheel_velocity = None

        # Previous slip angle for filtering
        self.prev_slip_angle = None

        # Integration method state
        self.slip_angle_integrated = 0.0
        self.last_time = None

        # Create a timer to publish drift state at a fixed rate
        self.create_timer(0.02, self.publish_drift_state)  # 50 Hz

        self.get_logger().info('Slip Estimator node started')
        self.get_logger().info(f'Estimation method: {self.estimation_method}')
        self.get_logger().info(f'Drift threshold: {math.degrees(self.drift_threshold):.1f} degrees')

    def imu_callback(self, msg):
        """Store latest IMU data."""
        self.latest_imu = msg

    def odom_callback(self, msg):
        """Store latest odometry data."""
        self.latest_odom = msg

    def odom_gt_callback(self, msg):
        """Store latest ground truth odometry data."""
        self.latest_odom_gt = msg

    def wheel_speeds_callback(self, msg):
        """Store latest wheel speeds."""
        self.latest_wheel_speeds = msg

    def wheel_velocity_callback(self, msg):
        """Store latest wheel velocity estimate."""
        self.latest_wheel_velocity = msg

    def calculate_slip_angle_ground_truth(self, odom_gt, yaw):
        """
        Method 1: Ground Truth - Transform world velocities to vehicle frame

        Uses actual chassis motion from Gazebo simulation.
        NOT realistic for real car (no GPS gives you lateral velocity).
        """
        # Get vehicle frame velocities
        vx_car = odom_gt.twist.twist.linear.x
        vy_car = odom_gt.twist.twist.linear.y

        # Calculate slip angle
        if abs(vx_car) > 0.1:
            slip_angle = math.atan2(vy_car, vx_car)
        else:
            slip_angle = 0.0

        return slip_angle, vx_car, vy_car

    def calculate_slip_angle_integration(self, lateral_accel, vx, yaw_rate, dt):
        """
        Method 2: Integration - Real-time estimation using IMU

        Uses: β_dot = (ay / vx) - r
        Integrates over time to get β.

        Pros: Uses realistic sensors (IMU + wheel encoders)
        Cons: Drifts over time, needs periodic reset
        """
        # Avoid division by zero
        if abs(vx) < 0.1:
            return 0.0

        # Calculate slip angle rate: β_dot = (ay / vx) - r
        beta_dot = (lateral_accel / vx) - yaw_rate

        # Integrate
        self.slip_angle_integrated += beta_dot * dt

        # Reset integration when driving nearly straight (anti-drift)
        if abs(lateral_accel) < self.reset_threshold and abs(yaw_rate) < self.reset_threshold:
            self.slip_angle_integrated *= 0.95  # Slowly decay to zero

        return self.slip_angle_integrated

    def publish_drift_state(self):
        """Compute and publish drift state."""
        # Check if we have all required data
        if (self.latest_imu is None or
            self.latest_odom is None or
            self.latest_wheel_speeds is None):
            return

        # Get current time for integration
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Extract data from odometry (Ackermann - only has vx)
        odom = self.latest_odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        vx_ackermann = odom.twist.twist.linear.x

        # Extract data from IMU
        imu = self.latest_imu
        yaw = imu.yaw
        yaw_rate = imu.yaw_rate
        lateral_accel = imu.lateral_accel

        # Extract wheel speeds
        wheels = self.latest_wheel_speeds

        # Calculate slip angle based on selected method
        if self.estimation_method == 'ground_truth' and self.latest_odom_gt is not None:
            # Method 1: Ground Truth (for validation)
            slip_angle, vx, vy = self.calculate_slip_angle_ground_truth(
                self.latest_odom_gt, yaw
            )
        else:
            # Method 2: Integration (realistic)
            slip_angle = self.calculate_slip_angle_integration(
                lateral_accel, vx_ackermann, yaw_rate, dt
            )
            vx = vx_ackermann
            vy = vx * math.tan(slip_angle) if abs(vx) > 0.1 else 0.0

        # Calculate velocity magnitude
        v_magnitude = math.sqrt(vx * vx + vy * vy)

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
        drift_state.header.stamp = current_time.to_msg()
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
                f'DRIFTING! Slip angle: {math.degrees(slip_angle):.1f}° (vx={vx:.2f}, vy={vy:.2f})',
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
