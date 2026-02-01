# drift-car-sim
Drift car using Gazebo Harmonic simulator

#### DRIFT CAR ALIASES ####

Commands to move the car using only Gazebo Harmonic

alias car_go='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"'
alias car_stop='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"'
alias car_left='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.3}"'
alias car_right='gz topic -t "/model/drift_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: -0.3}"'
alias car_fast='gz topic -t "/model/drift_carcmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.0}"'

### ROS2 integration


**How to compile?**

```
cd drift_car_ws
colcon build --packages-select drift_car_bringup
source install/setup.bash
```

**How to launch the simulation + bridge?**

```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/drift_car_ws/src/drift_car_sim/models
ros2 launch drift_car_bringup simulation.launch.py
```

