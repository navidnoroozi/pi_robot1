# Fix — connect the ESP32 to the RPi4:
Plug the USB cable from the ESP32 into one of the RPi4's USB-A ports. Then verify the device appears:
```bash
bashls /dev/ttyUSB*
```
Expected: `/dev/ttyUSB0`

If you see `/dev/ttyUSB0`, relaunch bringup and all 5 nodes will start. 
If you see nothing, try a different USB port or USB cable — some cables are charge-only and do not carry data.
Also add the `pi` user to the `dialout` group so the serial node has permission:
```bash
bashsudo usermod -a -G dialout pi
newgrp dialout
```
Then reboot the RPi4:
```bash
sudo reboot
```
# How to read the Encoder (ESP32) logs:
What does `E 0 0 50000` conceptually mean?
`E 0 0 50000` means: Left wheel ticks = 0, Right wheel ticks = 0, elapsed time = 50000 microseconds (50ms).

# Robot runs longer/overshoots: wrong COUNTS_PER_REV
The robot travels further than commanded because the odometry underestimates distance. When the path follower thinks the robot is at the 1.4m waypoint, it has actually travelled further.

The cause: our assumed CPR of 937 (based on gearbox ratio 21.3:1) is too high. If the actual CPR is lower, `METRES_PER_TICK` is underestimated — every tick is computed as fewer metres than it really is, so the robot has to travel further before the position estimate reaches the waypoint.

Measure the true CPR in 3 steps:
**Step 1** — Mark exactly 1 metre on the floor. Place the robot wheels at the start mark.
**Step 2** — Stop bringup. Run the serial monitor:
```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
left_total, right_total = 0, 0
print('Push robot EXACTLY 1 metre forward by hand, then press Ctrl+C')
try:
    while True:
        line = s.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('E '):
            parts = line.split()
            left_total  += int(parts[1])
            right_total += int(parts[2])
            print(f'L={left_total:6d}  R={right_total:6d}', end='\r')
except KeyboardInterrupt:
    pass
print(f'\nFinal: L={left_total}  R={right_total}')
print(f'Average ticks for 1m: {(left_total+right_total)/2:.0f}')
s.close()
"
```
**Step 3** — Push the robot exactly 1 metre forward by hand (along a taped straight line). Press Ctrl+C. The printed average is your true `COUNTS_PER_REV`.
Update both files with the measured value:
In `firmware.ino`:
```cpp
#define COUNTS_PER_REV1  <measured_value>
#define COUNTS_PER_REV2  <measured_value>
```
In `serial_bridge_node.py`:
```python
COUNTS_PER_REV = <measured_value>
```
Re-upload firmware and rebuild the package.

# Robot starts without waiting for the user command ENTER
This is a known Python issue. The `input()` call in a daemon thread consumes stdin from the terminal, but when you launch the script right after pressing Enter at a shell prompt, that buffered newline is immediately consumed by `input()` — the robot starts before you can react.
The fix is to use a ROS2 topic trigger instead of stdin. Replace the `--mode local` start mechanism with a topic. Here is the corrected `_wait_for_keypress` method — replace it in `path_follower_node.py`:
```python
def _wait_for_keypress(self):
    """
    Instead of stdin (which has buffering issues),
    wait for a message on /path_start topic.
    Start the robot by running in another terminal:
        ros2 topic pub /path_start std_msgs/msg/Empty {} --once
    """
    from std_msgs.msg import Empty
    self.get_logger().info(
        '\n' + '='*55 +
        '\n  PATH FOLLOWER — Test 1 (local)' +
        '\n  Path: rectangle 1.4m × 0.9m' +
        '\n  Place robot at start position facing forward.' +
        '\n  Then in another terminal run:' +
        '\n    ros2 topic pub /path_start std_msgs/msg/Empty {} --once' +
        '\n' + '='*55)

    self.start_sub = self.create_subscription(
        Empty, '/path_start', self._start_callback, 10)

def _start_callback(self, msg):
    if not self.started:
        self.started = True
        self.get_logger().info('START received — beginning path')
```
Also remove the from `std_msgs.msg import Empty` import from the top of the file (it is not there yet) and add it. Then add `std_msgs` to the `exec_depend` list in `package.xml`:
```xml
<exec_depend>std_msgs</exec_depend>
```
Updated launch procedure for Test 1:
```bash
# Terminal 1 — bringup (unchanged)
ros2 launch robot_bringup robot_bringup.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200

# Terminal 2 — path follower (waits for topic)
source ~/.bashrc
python3 ~/path_follower_node.py --mode local

# Terminal 3 — when ready, place robot, then fire the trigger
ros2 topic pub /path_start std_msgs/msg/Empty {} --once
```
This decouples the trigger completely from stdin. The robot will not move until you explicitly publish the `/path_start` topic from Terminal 3, giving you full control over timing.

# Motor stopped at /cmd_vel = 0.2 m/s — USB power brownout
At 0.2 m/s the motors draw significantly more current than at 0.1 m/s. The ESP32 is powered from the RPi4's USB port. When motor current surges, if any power rail shares a ground with the ESP32 USB supply (even through parasitic coupling), the ESP32 can brownout and reset — dropping the serial connection. Relaunching bringup re-established the connection, which is why it started working again.

## Fix 
Add a 100µF or 470µF capacitor across the ESP32's 5V and GND pins.
This stabilises the supply during current spikes.

## Polarity 
Electrolytic capacitors are polarised. Connect the wrong way and they can fail or explode.
- Longer leg → 5V (the ESP32 VIN)
- Shorter leg / side with stripe → GND