std_msgs/Header header

# Individual wheel speeds (angular velocity)
float64 front_left      # rad/s
float64 front_right     # rad/s
float64 rear_left       # rad/s
float64 rear_right      # rad/s

# Derived values
float64 average_rear    # rad/s - for velocity estimation
