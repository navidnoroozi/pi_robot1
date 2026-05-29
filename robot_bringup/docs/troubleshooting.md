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
