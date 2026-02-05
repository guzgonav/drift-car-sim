std_msgs/Header header

# Individual wheel speeds (angular velocity)
float64 front_left      # rad/s
float64 front_right     # rad/s
float64 rear_left       # rad/s
float64 rear_right      # rad/s

# Derived values
float64 average_rear    # rad/s - for velocity estimation

# Steering angles (from front steering joints)
float64 steering_angle_left   # rad - front left steering joint
float64 steering_angle_right  # rad - front right steering joint
float64 steering_angle        # rad - equivalent bicycle model angle (average)
