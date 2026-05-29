# Terminal 1:
```bash
sudo usermod -a -G dialout pi
newgrp dialout
source ~/.bashrc
cd ~/robot_ws
colcon build --packages-select robot_bringup
source install/setup.bash

ros2 launch robot_bringup robot_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200
```

# Terminal 2: drive forward for 1 metre
```bash
source ~/.bashrc
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

# Terminal 3: logs
```bash
source ~/.bashrc
ros2 topic list
ros2 topic echo /odom/unfiltered --qos-reliability best_effort --once
ros2 topic echo /odom --once
```
