#!/usr/bin/env python3
"""
IMU Processor Node

Subscribes to raw IMU data and processes it into a more useful format.
- Converts quaternion orientation to Euler angles
- Extracts yaw rate and lateral acceleration
- Applies optional low-pass filtering
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from drift_msgs.msg import ImuProcessed
from drift_state_estimation.utils.transforms import quaternion_to_euler
from drift_state_estimation.utils.filters import low_pass_filter


class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Declare parameters
        self.declare_parameter('filter_alpha', 0.1)

        # Get parameters
        self.filter_alpha = self.get_parameter('filter_alpha').value

        # Publisher
        self.pub_processed = self.create_publisher(
            ImuProcessed,
            '/drift/imu_processed',
            10
        )

        # Subscriber
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Previous values for filtering
        self.prev_yaw_rate = None
        self.prev_lateral_accel = None

        self.get_logger().info('IMU Processor node started')
        self.get_logger().info(f'Filter alpha: {self.filter_alpha}')

    def imu_callback(self, msg):
        """Process incoming IMU data."""
        # Convert quaternion to Euler angles
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Extract yaw rate (angular velocity around z-axis)
        yaw_rate = msg.angular_velocity.z

        # Extract lateral acceleration (y-axis in body frame)
        lateral_accel = msg.linear_acceleration.y

        # Apply low-pass filter
        yaw_rate = low_pass_filter(yaw_rate, self.prev_yaw_rate, self.filter_alpha)
        lateral_accel = low_pass_filter(lateral_accel, self.prev_lateral_accel, self.filter_alpha)

        # Store for next iteration
        self.prev_yaw_rate = yaw_rate
        self.prev_lateral_accel = lateral_accel

        # Create and publish processed message
        processed = ImuProcessed()
        processed.header = msg.header
        processed.roll = roll
        processed.pitch = pitch
        processed.yaw = yaw
        processed.yaw_rate = yaw_rate
        processed.lateral_accel = lateral_accel

        self.pub_processed.publish(processed)


def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
