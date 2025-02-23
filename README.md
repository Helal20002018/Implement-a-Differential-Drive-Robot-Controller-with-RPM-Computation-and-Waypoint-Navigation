# Differential Drive Robot Controller with Waypoint Navigation

## Overview
This project implements a ROS 2-based differential drive robot controller that computes wheel RPM from velocity commands and a waypoint navigation system using a PID controller.

## Features
- Computes wheel RPM from `/cmd_vel` commands.
- Publishes RPM values for motor control.
- Uses a PID controller to navigate between waypoints.
- Compatible with ROS 2 Humble and Gazebo Harmonic.

## Installation
```sh
colcon build --packages-select car_nav2
source install/setup.bash
```

## Usage
1. **Run the RPM Publisher:**
   ```sh
   ros2 run car_nav2 rpm_publisher
   ```
2. **Run the Waypoint Navigator:**
   ```sh
   ros2 run car_nav2 waypoint_navigation.py --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
   ```

## Topics
- `/cmd_vel` (Input): Velocity commands.
- `/left_wheel_rpm`, `/right_wheel_rpm` (Output): Computed RPM values.
- `/odom` (Input): Odometry data for waypoint navigation.

## Parameters
- `wheelbase`: Distance between wheels.
- `wheel_radius`: Wheel radius.
- `max_rpm`: Maximum RPM limit.
- `kp`, `ki`, `kd`: PID controller gains.

## Testing
```sh
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
ros2 topic echo /left_wheel_rpm
ros2 topic echo /right_wheel_rpm
```

## License
MIT License

