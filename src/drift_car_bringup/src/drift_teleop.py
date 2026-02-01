#!/usr/bin/env python3
"""
Simple keyboard teleoperation for drift car

Controls:
    W/↑    - Forward
    S/↓    - Backward
    A/←    - Steer left
    D/→    - Steer right
    Space  - Stop
    Q      - Quit
    1-5    - Speed levels
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriftTeleopNode(Node):
    def __init__(self):
        super().__init__('drift_teleop')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control parameters
        self.linear_speed = 1.5  # m/s
        self.angular_speed = 0.8  # rad/s

        # Current command
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info('Drift Car Teleop Started!')
        self.print_instructions()

    def print_instructions(self):
        msg = """
╔════════════════════════════════════════════╗
║      DRIFT CAR KEYBOARD CONTROL            ║
╠════════════════════════════════════════════╣
║  W/↑  : Forward    |  S/↓  : Backward      ║
║  A/←  : Left       |  D/→  : Right         ║
║  Space: Stop       |  Q    : Quit          ║
║                                            ║
║  1-5  : Speed levels (1=slow, 5=fast)     ║
║                                            ║
║  Current Speed: {:.1f} m/s                 ║
╚════════════════════════════════════════════╝
        """.format(self.linear_speed)
        print(msg)

    def publish_cmd_vel(self):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)

    def get_key(self):
        """Get keyboard input (non-blocking)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input"""
        if key is None:
            return True

        # Check for arrow keys
        if key == '\x1b':  # ESC sequence
            next1 = sys.stdin.read(1)
            next2 = sys.stdin.read(1)
            if next1 == '[':
                if next2 == 'A':  # Up arrow
                    key = 'w'
                elif next2 == 'B':  # Down arrow
                    key = 's'
                elif next2 == 'C':  # Right arrow
                    key = 'd'
                elif next2 == 'D':  # Left arrow
                    key = 'a'

        # Movement controls
        if key.lower() == 'w':
            self.linear_vel = self.linear_speed
            self.get_logger().info(f'Forward: {self.linear_vel:.1f} m/s')

        elif key.lower() == 's':
            self.linear_vel = -self.linear_speed
            self.get_logger().info(f'Backward: {self.linear_vel:.1f} m/s')

        elif key.lower() == 'a':
            self.angular_vel = self.angular_speed
            self.get_logger().info(f'Left: {self.angular_vel:.1f} rad/s')

        elif key.lower() == 'd':
            self.angular_vel = -self.angular_speed
            self.get_logger().info(f'Right: {self.angular_vel:.1f} rad/s')

        # Stop
        elif key == ' ':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.get_logger().info('Stopped')

        # Speed adjustment
        elif key in ['1', '2', '3', '4', '5']:
            speed_level = int(key)
            self.linear_speed = 0.5 * speed_level
            self.angular_speed = 0.3 + (0.15 * speed_level)
            self.get_logger().info(
                f'Speed Level {speed_level}: Linear={self.linear_speed:.1f} m/s, '
                f'Angular={self.angular_speed:.1f} rad/s'
            )

        # Quit
        elif key.lower() == 'q':
            self.get_logger().info('Shutting down teleop...')
            return False

        return True

    def run(self):
        """Main run loop"""
        # Save terminal settings
        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())

            while rclpy.ok():
                key = self.get_key()
                if not self.process_key(key):
                    break

                # Let ROS process callbacks
                rclpy.spin_once(self, timeout_sec=0)

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            # Stop the car
            msg = Twist()
            self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriftTeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
