# drift-car-sim
Drift car using Gazebo Harmonic simulator

#### DRIFT CAR ALIASES ####

alias car_go='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"'
alias car_stop='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"'
alias car_left='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.3}"'
alias car_right='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: -0.3}"'
alias car_fast='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.0}"'
