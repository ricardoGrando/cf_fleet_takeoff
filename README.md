# cf_fleet_takeoff

A tiny ROS 2 (rclpy) package that commands a fleet of Crazyflies to **take off and hover at 1.0 m**, using:
- `/<uav>/cmd_vel` (geometry_msgs/Twist) for vertical velocity
- `/<uav>/odom` (nav_msgs/Odometry) for altitude feedback

It assumes you already have per-UAV bridges in place:
- `/<uav>/cmd_vel  →  /<uav>/gazebo/command/twist` (ROS → GZ)
- `/model/<uav>/odometry  →  /<uav>/odom` (GZ → ROS)

## Build
```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src
# place this folder here as cf_fleet_takeoff/
cd ..
colcon build --symlink-install
source install/setup.bash

