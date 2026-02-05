#!/usr/bin/env python3
"""
Wheel Odometry Node

Processes joint states to extract wheel speeds and calculate expected vehicle velocity.
Assumes no slip (for comparison with actual velocity during drift detection).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from drift_msgs.msg import WheelSpeeds


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheelbase', 0.26)
        self.declare_parameter('track_width', 0.18)

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value

        # Publishers
        self.pub_wheel_speeds = self.create_publisher(
            WheelSpeeds,
            '/drift/wheel_speeds',
            10
        )
        self.pub_wheel_velocity = self.create_publisher(
            TwistStamped,
            '/drift/wheel_velocity',
            10
        )

        # Subscriber
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.get_logger().info('Wheel Odometry node started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')

    def joint_states_callback(self, msg):
        """Process joint states to extract wheel speeds and steering angles."""
        # Find wheel joint indices
        try:
            idx_fl = msg.name.index('front_left_wheel_joint')
            idx_fr = msg.name.index('front_right_wheel_joint')
            idx_rl = msg.name.index('rear_left_wheel_joint')
            idx_rr = msg.name.index('rear_right_wheel_joint')
        except ValueError:
            self.get_logger().warn('Wheel joints not found in joint states', throttle_duration_sec=5.0)
            return

        # Find steering joint indices
        try:
            idx_steer_fl = msg.name.index('front_left_steering_joint')
            idx_steer_fr = msg.name.index('front_right_steering_joint')
        except ValueError:
            self.get_logger().warn('Steering joints not found in joint states', throttle_duration_sec=5.0)
            idx_steer_fl = None
            idx_steer_fr = None

        # Extract wheel angular velocities (rad/s)
        wheel_speed_fl = msg.velocity[idx_fl] if idx_fl < len(msg.velocity) else 0.0
        wheel_speed_fr = msg.velocity[idx_fr] if idx_fr < len(msg.velocity) else 0.0
        wheel_speed_rl = msg.velocity[idx_rl] if idx_rl < len(msg.velocity) else 0.0
        wheel_speed_rr = msg.velocity[idx_rr] if idx_rr < len(msg.velocity) else 0.0

        # Extract steering angles (rad) from joint positions
        if idx_steer_fl is not None and idx_steer_fl < len(msg.position):
            steering_angle_left = msg.position[idx_steer_fl]
        else:
            steering_angle_left = 0.0

        if idx_steer_fr is not None and idx_steer_fr < len(msg.position):
            steering_angle_right = msg.position[idx_steer_fr]
        else:
            steering_angle_right = 0.0

        # Calculate equivalent bicycle model steering angle (average of both)
        # Note: In Ackermann, inner wheel turns more than outer
        steering_angle = (steering_angle_left + steering_angle_right) / 2.0

        # Calculate average rear wheel speed
        average_rear = (wheel_speed_rl + wheel_speed_rr) / 2.0

        # Publish wheel speeds
        wheel_speeds = WheelSpeeds()
        wheel_speeds.header.stamp = self.get_clock().now().to_msg()
        wheel_speeds.front_left = wheel_speed_fl
        wheel_speeds.front_right = wheel_speed_fr
        wheel_speeds.rear_left = wheel_speed_rl
        wheel_speeds.rear_right = wheel_speed_rr
        wheel_speeds.average_rear = average_rear
        wheel_speeds.steering_angle_left = steering_angle_left
        wheel_speeds.steering_angle_right = steering_angle_right
        wheel_speeds.steering_angle = steering_angle

        self.pub_wheel_speeds.publish(wheel_speeds)

        # Calculate expected forward velocity (assumes no slip)
        # v = omega * radius
        expected_velocity = average_rear * self.wheel_radius

        # Publish expected velocity as Twist
        twist = TwistStamped()
        twist.header.stamp = wheel_speeds.header.stamp
        twist.twist.linear.x = expected_velocity
        twist.twist.linear.y = 0.0  # No lateral velocity assumed
        twist.twist.linear.z = 0.0

        self.pub_wheel_velocity.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
