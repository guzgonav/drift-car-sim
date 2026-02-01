# drift-car-sim
Drift car using Gazebo Harmonic simulator

#### DRIFT CAR ALIASES ####

# Drive
alias car_forward='gz topic -t "/model/drift_car/joint/rear_left_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 10.0" && gz topic -t "/model/drift_car/joint/rear_right_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 10.0"'

alias car_reverse='gz topic -t "/model/drift_car/joint/rear_left_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: -10.0" && gz topic -t "/model/drift_car/joint/rear_right_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: -10.0"'

alias car_stop='gz topic -t "/model/drift_car/joint/rear_left_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 0.0" && gz topic -t "/model/drift_car/joint/rear_right_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 0.0"'

# Steering
alias car_left='gz topic -t "/model/drift_car/joint/front_left_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: 2" && gz topic -t "/model/drift_car/joint/front_right_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: 2"'

alias car_right='gz topic -t "/model/drift_car/joint/front_left_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: -1" && gz topic -t "/model/drift_car/joint/front_right_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: -1"'

alias car_straight='gz topic -t "/model/drift_car/joint/front_left_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: 0.1" && gz topic -t "/model/drift_car/joint/front_right_steering_joint/0/cmd_pos" -m gz.msgs.Double -p "data: 0.1"'

# Fast speed
alias car_fast='gz topic -t "/model/drift_car/joint/rear_left_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 30.0" && gz topic -t "/model/drift_car/joint/rear_right_wheel_joint/cmd_vel" -m gz.msgs.Double -p "data: 30.0"'
