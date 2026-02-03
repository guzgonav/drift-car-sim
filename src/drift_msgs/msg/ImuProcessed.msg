std_msgs/Header header

# Orientation (Euler angles)
float64 roll            # rad
float64 pitch           # rad
float64 yaw             # rad

# Angular velocities
float64 yaw_rate        # rad/s - CRITICAL for drift

# Linear accelerations
float64 lateral_accel   # m/s^2 - CRITICAL for drift
