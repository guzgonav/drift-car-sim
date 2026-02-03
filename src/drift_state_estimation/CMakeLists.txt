cmake_minimum_required(VERSION 3.8)
project(drift_state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  drift_state_estimation/imu_processor.py
  drift_state_estimation/wheel_odometry.py
  drift_state_estimation/slip_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
