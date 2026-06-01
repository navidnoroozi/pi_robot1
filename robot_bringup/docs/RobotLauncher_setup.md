# General Workflow:
```
Deploy/build package → launch robot → start recording → send /cmd_vel → stop robot → stop recording → run plot_bag.py → copy plots to Ubuntu VM
```

# One-time preparation on the RPi4
Install bag/plot dependencies if not already installed:
```bash
sudo apt update
sudo apt install ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap python3-matplotlib python3-yaml
```

## Deploy/build your ROS 2 package on the RPi4
Build your pkg
```bash
sudo usermod -a -G dialout pi
newgrp dialout
source ~/.bashrc
cd ~/robot_ws
colcon build --packages-select robot_bringup
source install/setup.bash
```
**Good practice**: 
Check that ROS 2 can see your package:
```bash
ros2 pkg list | grep robot_bringup
```
Check that the launch file is visible:
```bash
ros2 launch robot_bringup robot_bringup.launch.py --show-args
```
If this works, your package is deployed and buildable.


# Regular workflow
## Terminal 1: launch robot bringup
```bash
sudo usermod -a -G dialout pi
newgrp dialout
source ~/.bashrc
cd ~/robot_ws
source install/setup.bash
```
## Launch robot bringup
```bash
ros2 launch robot_bringup robot_bringup.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200
```

## Terminal 2: topic checks and logs
```bash
source ~/.bashrc
cd ~/robot_ws
source install/setup.bash
```
Check that the important topics exist:
```bash
ros2 topic list
```
If `/odom` and `/imu/data` are publishing, the robot is alive and ready for commandin.
```bash
ros2 topic echo /odom/unfiltered --qos-reliability best_effort --once
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /cmd_vel --once
```
Go to Terminal 2 for commanding.

⏺️**Recording Data using DataLogger**: 👉Check out `DataLogger_setup.md`

## Terminal 3: drive forward for 1 metre
**To drive approximately 1 metre at 0.1 m/s, the ideal duration is:**
```
distance = velocity × time
1.0 m = 0.1 m/s × 10 s
```
So publish /cmd_vel for 10 seconds:
```bash
source ~/.bashrc
cd ~/robot_ws
source install/setup.bash
timeout 10s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```
Then send a zero velocity command to stop the robot:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```
**Alternative Approach**: send a permanent velocity commands for driving forward
```bash
source ~/.bashrc
cd ~/robot_ws
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```