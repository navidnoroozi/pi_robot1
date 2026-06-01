#!/usr/bin/env python3
"""
serial_bridge_node.py — Approach 1 custom serial bridge
---------------------------------------------------------
Runs on the RPi4. Bridges between the ESP32 serial firmware
and ROS2 topics:

  Subscribes:  /cmd_vel  (geometry_msgs/Twist)
  Publishes:   /odom/unfiltered  (nav_msgs/Odometry)

Serial protocol with ESP32:
  Send:    "W <left_rpm> <right_rpm>\\n"
  Receive: "E <left_ticks> <right_ticks> <dt_us>\\n"

Differential drive kinematics:
  v_left, v_right (m/s) from ticks and dt
  v     = (v_right + v_left) / 2
  omega = (v_right - v_left) / wheel_separation
  x    += v * cos(theta) * dt
  y    += v * sin(theta) * dt
  theta+= omega * dt
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


# ── Robot physical parameters ────────────────────────────────────────────────
# COUNTS_PER_REV    = 1001      # 11 CPR × 21.3 gear ratio × 4 quadrature = 937, but measured 1001 ticks/rev
WHEEL_DIAMETER    = 0.065    # metres (65 mm wheel)
WHEEL_RADIUS      = WHEEL_DIAMETER / 2.0
WHEEL_SEPARATION  = 0.170    # metres — centre to centre of wheels
MOTOR_MAX_RPM     = 259      # 280 RPM × (11.4V / 12V)

# METRES_PER_TICK comes directly from calibration (1m push test):
# L=1009 ticks, R=992 ticks → average 1001 ticks per metre
# Using this directly avoids confusion between ticks/rev and ticks/m
METRES_PER_TICK = 1.0 / 1001.0


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ── Parameters (can be overridden from launch file) ──────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)

        port      = self.get_parameter('serial_port').value
        baud      = self.get_parameter('baud_rate').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ── Odometry state ───────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Serial port ──────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # ── QoS: BEST_EFFORT matches EKF subscription ────────────────────────
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(
            Odometry, '/odom/unfiltered', best_effort_qos)

        # ── TF broadcaster ───────────────────────────────────────────────────
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ── Serial read thread ───────────────────────────────────────────────
        self._serial_lock = threading.Lock()
        self._read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info('Serial bridge node started')

    # ── /cmd_vel callback ────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist (linear.x m/s, angular.z rad/s) to
        left/right wheel RPM and send to ESP32.

        Differential drive inverse kinematics:
          v_left  = linear_x - angular_z * (wheel_separation / 2)
          v_right = linear_x + angular_z * (wheel_separation / 2)
          rpm     = (v_m/s / wheel_circumference) * 60
        """
        v   = msg.linear.x
        omega = msg.angular.z

        v_left  = v - omega * (WHEEL_SEPARATION / 2.0)
        v_right = v + omega * (WHEEL_SEPARATION / 2.0)

        # Convert m/s → RPM
        rpm_left  = (v_left  / (math.pi * WHEEL_DIAMETER)) * 60.0
        rpm_right = (v_right / (math.pi * WHEEL_DIAMETER)) * 60.0

        # Clamp to motor limits
        rpm_left  = max(-MOTOR_MAX_RPM, min(MOTOR_MAX_RPM, rpm_left))
        rpm_right = max(-MOTOR_MAX_RPM, min(MOTOR_MAX_RPM, rpm_right))

        cmd = f"W {rpm_left:.2f} {rpm_right:.2f}\n"
        try:
            with self._serial_lock:
                self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial read loop (runs in background thread) ─────────────────────────
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
        Parse "E <left_ticks> <right_ticks> <dt_us>"
        and publish odometry.
        """
        if not line.startswith('E '):
            return

        parts = line.split()
        if len(parts) < 4:
            return

        try:
            left_ticks  = int(parts[1])
            right_ticks = int(parts[2])
            dt_us       = int(parts[3])
        except ValueError:
            return

        if dt_us <= 0:
            return

        dt_s = dt_us * 1e-6

        # ── Differential drive forward kinematics ────────────────────────────
        d_left  = left_ticks  * METRES_PER_TICK
        d_right = right_ticks * METRES_PER_TICK

        d_centre = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / WHEEL_SEPARATION

        # Update pose
        self.theta += d_theta
        # Normalise angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += d_centre * math.cos(self.theta)
        self.y += d_centre * math.sin(self.theta)

        # ── Velocities for this interval ─────────────────────────────────────
        v_linear  = d_centre / dt_s
        v_angular = d_theta  / dt_s

        # ── Build and publish Odometry message ───────────────────────────────
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation as quaternion (rotation around Z only)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Pose covariance (6×6 diagonal, row-major)
        odom.pose.covariance[0]  = 0.001   # x
        odom.pose.covariance[7]  = 0.001   # y
        odom.pose.covariance[35] = 0.01    # yaw

        # Twist (velocities in child frame = base_footprint)
        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        # ── Publish odom → base_footprint TF ─────────────────────────────────
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame

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
