#!/usr/bin/env python3
"""
Simple keyboard teleop for drift car in Gazebo Sim.

Controls:
    W / UP      - Forward
    S / DOWN    - Reverse
    A / LEFT    - Steer left
    D / RIGHT   - Steer right
    SPACE       - Stop and straighten
    C           - Center steering only
    Q           - Quit
"""

import subprocess
import sys
import time
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Installing pynput...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pynput"])
    from pynput import keyboard


class DriftCarTeleop:
    def __init__(self):
        # State
        self.throttle = 0.0
        self.steering = 0.0
        
        # Parameters
        self.max_throttle = 15.0
        self.max_steering = 0.5
        
        # Topics
        self.rear_left_topic = "/model/drift_car/joint/rear_left_wheel_joint/cmd_vel"
        self.rear_right_topic = "/model/drift_car/joint/rear_right_wheel_joint/cmd_vel"
        self.steer_left_topic = "/model/drift_car/joint/front_left_steering_joint/0/cmd_pos"
        self.steer_right_topic = "/model/drift_car/joint/front_right_steering_joint/0/cmd_pos"
        
        # Track pressed keys using simple flags
        self.forward_pressed = False
        self.reverse_pressed = False
        self.left_pressed = False
        self.right_pressed = False
        
        # Running flag
        self.running = True
        
    def send_gz_command(self, topic, value):
        """Send a command to Gazebo via gz topic."""
        cmd = [
            "gz", "topic", "-t", topic,
            "-m", "gz.msgs.Double",
            "-p", f"data: {value}"
        ]
        try:
            subprocess.run(cmd, capture_output=True, timeout=0.5)
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"\nError sending command: {e}")
    
    def update_throttle(self):
        """Send throttle commands to both rear wheels."""
        self.send_gz_command(self.rear_left_topic, self.throttle)
        self.send_gz_command(self.rear_right_topic, self.throttle)
    
    def update_steering(self):
        """Send steering commands to both front wheels."""
        self.send_gz_command(self.steer_left_topic, self.steering)
        self.send_gz_command(self.steer_right_topic, self.steering)
    
    def get_key_name(self, key):
        """Convert key to a simple string identifier."""
        try:
            if hasattr(key, 'char') and key.char:
                return key.char.lower()
        except AttributeError:
            pass
        
        if key == keyboard.Key.up:
            return 'up'
        elif key == keyboard.Key.down:
            return 'down'
        elif key == keyboard.Key.left:
            return 'left'
        elif key == keyboard.Key.right:
            return 'right'
        elif key == keyboard.Key.space:
            return 'space'
        return str(key)
    
    def on_press(self, key):
        """Handle key press events."""
        k = self.get_key_name(key)
        
        # Throttle control
        if k in ['w', 'up']:
            self.forward_pressed = True
            self.throttle = self.max_throttle
            self.update_throttle()
            
        elif k in ['s', 'down']:
            self.reverse_pressed = True
            self.throttle = -self.max_throttle
            self.update_throttle()
            
        # Steering control
        elif k in ['a', 'left']:
            self.left_pressed = True
            self.steering = self.max_steering
            self.update_steering()
            
        elif k in ['d', 'right']:
            self.right_pressed = True
            self.steering = -self.max_steering
            self.update_steering()
            
        # Full stop
        elif k == 'space':
            self.throttle = 0.0
            self.steering = 0.0
            self.forward_pressed = False
            self.reverse_pressed = False
            self.left_pressed = False
            self.right_pressed = False
            self.update_throttle()
            self.update_steering()
        
        # Center steering only
        elif k == 'c':
            self.steering = 0.0
            self.left_pressed = False
            self.right_pressed = False
            self.update_steering()
            
        # Quit
        elif k == 'q':
            self.throttle = 0.0
            self.steering = 0.0
            self.update_throttle()
            self.update_steering()
            self.running = False
            return False
    
    def on_release(self, key):
        """Handle key release events."""
        k = self.get_key_name(key)
        
        # Release throttle
        if k in ['w', 'up']:
            self.forward_pressed = False
            if not self.reverse_pressed:
                self.throttle = 0.0
                self.update_throttle()
                
        elif k in ['s', 'down']:
            self.reverse_pressed = False
            if not self.forward_pressed:
                self.throttle = 0.0
                self.update_throttle()
        
        # Release steering - return to center
        elif k in ['a', 'left']:
            self.left_pressed = False
            if not self.right_pressed:
                self.steering = 0.0
                self.update_steering()
                
        elif k in ['d', 'right']:
            self.right_pressed = False
            if not self.left_pressed:
                self.steering = 0.0
                self.update_steering()
    
    def print_status(self):
        """Print current status."""
        while self.running:
            status = f"\rThrottle: {self.throttle:+6.1f} | Steering: {self.steering:+5.2f} | [SPACE]stop [C]enter [Q]uit"
            print(status, end="", flush=True)
            time.sleep(0.1)
    
    def run(self):
        """Main run loop."""
        print("=" * 65)
        print("Drift Car Teleop")
        print("=" * 65)
        print("Controls:")
        print("  W / ↑     - Forward")
        print("  S / ↓     - Reverse")
        print("  A / ←     - Steer left")
        print("  D / →     - Steer right")
        print("  C         - Center steering")
        print("  SPACE     - Full stop (throttle + steering)")
        print("  Q         - Quit")
        print("=" * 65)
        print("\nKeep this terminal focused for keyboard input!\n")
        
        # Start status thread
        status_thread = threading.Thread(target=self.print_status, daemon=True)
        status_thread.start()
        
        # Start keyboard listener
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()
        
        print("\n\nStopped.")


def main():
    teleop = DriftCarTeleop()
    try:
        teleop.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        teleop.throttle = 0.0
        teleop.steering = 0.0
        teleop.update_throttle()
        teleop.update_steering()


if __name__ == "__main__":
    main()