#!/usr/bin/env python3
"""
serial_bridge_node.py — custom serial bridge for a 2WD differential-drive robot
-----------------------------------------------------------------------------

Runs on the RPi4. Bridges between the ESP32 serial firmware and ROS2 topics:

  Subscribes:  /cmd_vel            (geometry_msgs/Twist)
  Publishes:   /odom/unfiltered    (nav_msgs/Odometry)

Serial protocol with ESP32:f
  Send:    "W <left_rpm> <right_rpm>\n"
  Receive: "E <left_ticks> <right_ticks> <dt_us> ...\n"

Important calibration convention:
  The ticks coming from the ESP32 "E ..." line are already decoded ticks from
  the firmware's quadrature ISR. Do not multiply them by 4 again here.
"""

import math
import serial
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ── Default robot physical parameters ────────────────────────────────────────
DEFAULT_WHEEL_DIAMETER = 0.065       # metres
DEFAULT_WHEEL_SEPARATION = 0.170     # metres, wheel-centre to wheel-centre
DEFAULT_MOTOR_MAX_RPM = 259.0

# Calibration from the 1 m push test. These are ticks per metre from the ESP32
# E message, which is already 4× quadrature decoded by the firmware.
DEFAULT_LEFT_TICKS_PER_METER = 1001.0 # 1009.0
DEFAULT_RIGHT_TICKS_PER_METER = 1001.0 #992.0


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ── Parameters (can be overridden from launch file) ──────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)

        # Calibration / geometry parameters
        self.declare_parameter('wheel_diameter', DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter('wheel_separation', DEFAULT_WHEEL_SEPARATION)
        self.declare_parameter('motor_max_rpm', DEFAULT_MOTOR_MAX_RPM)
        self.declare_parameter('left_ticks_per_meter', DEFAULT_LEFT_TICKS_PER_METER)
        self.declare_parameter('right_ticks_per_meter', DEFAULT_RIGHT_TICKS_PER_METER)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.motor_max_rpm = float(self.get_parameter('motor_max_rpm').value)
        self.left_ticks_per_meter = float(self.get_parameter('left_ticks_per_meter').value)
        self.right_ticks_per_meter = float(self.get_parameter('right_ticks_per_meter').value)

        if self.left_ticks_per_meter <= 0.0 or self.right_ticks_per_meter <= 0.0:
            raise ValueError('left_ticks_per_meter and right_ticks_per_meter must be positive')
        if self.wheel_diameter <= 0.0 or self.wheel_separation <= 0.0:
            raise ValueError('wheel_diameter and wheel_separation must be positive')

        self.left_metres_per_tick = 1.0 / self.left_ticks_per_meter
        self.right_metres_per_tick = 1.0 / self.right_ticks_per_meter

        # ── Odometry state ───────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ── Serial port ──────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odom/unfiltered', best_effort_qos)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self._serial_lock = threading.Lock()
        self._read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            'Serial bridge node started with '
            f'left_ticks_per_meter={self.left_ticks_per_meter:.1f}, '
            f'right_ticks_per_meter={self.right_ticks_per_meter:.1f}, '
            f'wheel_diameter={self.wheel_diameter:.3f} m, '
            f'wheel_separation={self.wheel_separation:.3f} m'
        )

    # ── /cmd_vel callback ────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist (linear.x m/s, angular.z rad/s) to left/right wheel RPM
        and send it to the ESP32 firmware.

        Differential-drive inverse kinematics:
          v_left  = v - omega * wheel_separation / 2
          v_right = v + omega * wheel_separation / 2
          rpm     = v_wheel / wheel_circumference * 60
        """
        v = msg.linear.x
        omega = msg.angular.z

        v_left = v - omega * (self.wheel_separation / 2.0)
        v_right = v + omega * (self.wheel_separation / 2.0)

        wheel_circumference = math.pi * self.wheel_diameter
        rpm_left = (v_left / wheel_circumference) * 60.0
        rpm_right = (v_right / wheel_circumference) * 60.0

        rpm_left = max(-self.motor_max_rpm, min(self.motor_max_rpm, rpm_left))
        rpm_right = max(-self.motor_max_rpm, min(self.motor_max_rpm, rpm_right))

        cmd = f"W {rpm_left:.2f} {rpm_right:.2f}\n"
        try:
            with self._serial_lock:
                self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial read loop ─────────────────────────────────────────────────────
    def _serial_read_loop(self):
        buf = ''
        while rclpy.ok():
            try:
                data = self.ser.read(256).decode('ascii', errors='ignore')
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                continue

            buf += data
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                self._parse_encoder_line(line.strip())

    # ── Parse encoder report from ESP32 ─────────────────────────────────────
    def _parse_encoder_line(self, line: str):
        """
        Parse "E <left_ticks> <right_ticks> <dt_us> ..."
        and publish odometry.

        The first three numeric fields are used. Extra firmware debug fields are
        ignored, so the firmware may also append target RPMs, measured RPMs, etc.
        """
        if not line.startswith('E '):
            return

        parts = line.split()
        if len(parts) < 4:
            return

        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
            dt_us = int(parts[3])
        except ValueError:
            return

        if dt_us <= 0:
            return

        dt_s = dt_us * 1e-6

        # Encoder ticks from firmware are already quadrature decoded.
        d_left = left_ticks * self.left_metres_per_tick
        d_right = right_ticks * self.right_metres_per_tick

        d_centre = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Use midpoint integration for a differential drive.
        # theta_mid = self.theta + 0.5 * d_theta
        # self.x += d_centre * math.cos(theta_mid)
        # self.y += d_centre * math.sin(theta_mid)
        # self.theta += d_theta
        
        self.x += d_centre * math.cos(self.theta)
        self.y += d_centre * math.sin(self.theta)
        self.theta += d_theta
        # Normalise angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        v_linear = d_centre / dt_s
        v_angular = d_theta / dt_s

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.01

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = math.sin(self.theta / 2.0)
            tf.transform.rotation.w = math.cos(self.theta / 2.0)

            self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        """Send stop command before shutting down."""
        try:
            self.ser.write(b"S\n")
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
