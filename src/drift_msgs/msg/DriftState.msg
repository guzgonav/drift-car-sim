std_msgs/Header header

# Pose (from odometry)
float64 x               # m
float64 y               # m
float64 yaw             # rad

# Velocities
float64 vx              # forward velocity (m/s)
float64 vy              # lateral velocity (m/s)
float64 v_magnitude     # total speed sqrt(vx^2 + vy^2)
float64 yaw_rate        # rad/s

# THE KEY METRICS FOR DRIFTING
float64 slip_angle      # β (rad) - THE MOST IMPORTANT VALUE
float64 lateral_accel   # m/s^2

# Wheel data
float64 wheel_speed_fl  # rad/s
float64 wheel_speed_fr  # rad/s
float64 wheel_speed_rl  # rad/s
float64 wheel_speed_rr  # rad/s

# Steering
float64 steering_angle  # rad

# Status
bool is_drifting        # true if |slip_angle| > threshold
