Here is the instructions to use `plot_bag.py` on the `RPi4` to record the logs and plot them offline:

# Background
ROS 2 uses `ros2 bag` to record data from topics and later inspect or replay it. The `ros2 topic pub` command publishes a message to a topic using `YAML`-like syntax. ROS2 launch files are used to start several nodes and configurations together using `ros2 launch`.

## Disclaimer
`plot_bag.py` is **not** really a playback function. It is an **offline bag-analysis** script. 
You run it after recording has finished. It reads the saved `MCAP` bag and exports plots/images.
The `MCAP` storage plugin is the ROS2 bag storage plugin for the `MCAP` file format.

# Workflow
Here is a summary of the logs recording:
1. Terminal 1 on RPi4: launch robot bringup. 👉Check out `RobotLauncher_setup.md` for details
2. Terminal 2 on RPi4: start recording. 👇See below for the details
3. Terminal 3 on RPi4: drive the robot. 👉Check out `RobotLauncher_setup.md` for details
4. Terminal 2: stop recording. 👇See below for the details
5. Offline Plotting: Run `plot_bag.py` on the RPi4. 👇See below for the details

## Terminal 2 on RPi4: start recording
1. For repeatable tests, first remove the old bag:
```bash
rm -r /home/pi/robot_ws/src/robot_bringup/bags/robot_run   # assuming the old data are stored under /home/pi/robot_ws/src/robot_bringup/bags/robot_run
```
2. Record the important topics:
```bash
ros2 bag record -o ~/robot_ws/src/robot_bringup/bags/robot_run /cmd_vel /odom /odom/unfiltered \
/imu/data \
/diagnostics \
/rosout \
/tf \
/tf_static \
/robot_description
```
**Note**: `-o` means option. It lets you choose the bag name instead of ROS2 generating a timestamped name automatically.
- **Alternatively**, you may recoend **all** existing topics (`-a` records all topics visible in the ROS2 system.)
```bash
ros2 bag record -a -o /tmp/robot_run
```
In **Jazzy**, the result became:
```bash
/tmp/robot_run/
├── metadata.yaml
└── robot_run_0.mcap
```cd ../../..

## Offline Plotting: Run plot_bag.py on the RPi4
Run `plot_bag.py` on the RPi4, not on the Ubuntu VM, because the bag is physically stored on the RPi4 under:
```bash
/tmp/robot_run
```
and because your script uses ROS2 Jazzy Python tools such as `rosbag2_py`.
1. On the RPi4 run `plot_bag.py`:
```bash
source ~/.bashrc
cd ~/robot_ws
source install/setup.bash
python3 ~/plot_bag.py
```
The script should export plots into something like:
```bash
/home/pi/bag_plots/
```

## Push/Copy the exported plots to the Ubuntu VM
To see the plots, switch to the Ubuntu VM terminal. 


**Alternatively**, use `scp` to copy manually the exports to the Ubuntu VM. Create a local folder:
```bash
mkdir -p ~/robot_run/bag_plots
```
Copy all exported plots:
```bash
scp pi@robot1:/home/pi/bag_plots/* ~/robot_run/bag_plots/
```
To open the folder graphically:
```bash
xdg-open ~/robot_run/bag_plots
```
