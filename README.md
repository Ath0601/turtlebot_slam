# DWA Planner
Implemented a custom DWA Planner using Python in ROS2 Humble and Turtlebot3 Waffle.

## Implementation
### Clone the repository
```
git clone https://github.com/Ath0601/turtlebot_slam.git
```

### Launch the simulation
```
ros2 launch dwa_planner nav.launch.py
```
The launch file includes launch the Ignition Gazebo, Turtlebot, its controller and the DWA Planner. The robot is equipped with a LiDAR, IMU for navigation.
The topics for these are bridged and these bridges are also launched along with the rest of the setup.
Nodes included in the launch file:
- DWA Planner
- Wheel Odometry
- Twist to Effort Converter: To convert cmd_vel Twist messages into Effort for the controller
- Goal Publisher

### Simulation Demo

https://github.com/user-attachments/assets/463660ee-42d3-43e8-a4b2-4804de95c09c

