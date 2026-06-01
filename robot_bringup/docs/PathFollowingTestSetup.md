# Terminal 1 — bringup
```bash
ros2 launch robot_bringup robot_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 baud_rate:=115200
```

# Terminal 2 — path follower (waits for topic trigger)
```bash
python3 ~/path_follower_node.py --mode local
```

# Wait for: "✓ Odom stable — ready to start"
# Place robot, then Terminal 3:
```bash
ros2 topic pub /path_start std_msgs/msg/Empty {} --once
```