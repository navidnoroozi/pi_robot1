#!/usr/bin/env python3
"""
path_follower_node.py — Waypoint path follower for robot1
----------------------------------------------------------
Runs on the RPi4. Subscribes to /odom, publishes /cmd_vel.

USAGE — Test 1 (run entirely on RPi4):
    python3 path_follower_node.py --mode local

USAGE — Test 2 (triggered from Ubuntu VM via ZMQ):
    python3 path_follower_node.py --mode zmq --zmq-port 5560

The path is a rectangle that fits in a 3m × 2m lab space:
    (0,0) → (1.4,0) → (1.4,0.9) → (0,0.9) → (0,0)
    Total perimeter: 4.6 m, safely fits in the 3×2 area.

State machine:
    WAITING → ROTATING → DRIVING → WAYPOINT_REACHED → DONE
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

# ── Path definition ───────────────────────────────────────────────────────────
# Rectangle in the odom frame, starting at origin facing +X.
# Adjust these to fit your exact lab space and starting position.
WAYPOINTS = [
    (1.40, 0.00),   # drive forward 1.4 m
    (1.40, 0.90),   # turn left, drive 0.9 m
    (0.00, 0.90),   # turn left, drive back 1.4 m
    (0.00, 0.00),   # turn left, return to start
]

# ── Controller parameters ─────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.08   # m  — accept waypoint when within 8 cm
HEADING_TOLERANCE   = 0.10   # rad — ~5.7°, start driving when heading is close
MAX_LINEAR_SPEED    = 0.12   # m/s — safe for indoor, PID can track this well
MAX_ANGULAR_SPEED   = 0.50   # rad/s
KP_LINEAR           = 1.00   # proportional gain for forward speed
KP_ANGULAR          = 1.80   # proportional gain for heading correction
STOP_DURATION       = 1.0    # s  — pause at each waypoint

# ── State machine states ──────────────────────────────────────────────────────
WAITING  = 'WAITING'
ROTATING = 'ROTATING'
DRIVING  = 'DRIVING'
PAUSING  = 'PAUSING'
DONE     = 'DONE'


def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_to_yaw(qx, qy, qz, qw) -> float:
    """Extract yaw from quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class PathFollowerNode(Node):

    def __init__(self, mode: str, zmq_port: int):
        super().__init__('path_follower_node')

        self.mode     = mode
        self.started  = (mode == 'local')  # local mode starts immediately on trigger
        self.state    = WAITING
        self.wp_idx   = 0
        self.pause_t  = None

        # Current pose (updated from /odom)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.odom_received = False

        # ── Subscribers ────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos)

        # ── Publisher ──────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control loop at 20 Hz ──────────────────────────────────────────
        self.timer = self.create_timer(0.05, self._control_loop)

        # ── ZMQ start trigger (Test 2 only) ────────────────────────────────
        if mode == 'zmq':
            self._start_zmq_listener(zmq_port)

        self.get_logger().info(
            f'Path follower ready — mode={mode}, '
            f'{len(WAYPOINTS)} waypoints, '
            f'area fits 3m×2m lab')

        if mode == 'local':
            self._wait_for_keypress()

    # ── Odom callback ────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    # ── Main control loop ────────────────────────────────────────────────────
    def _control_loop(self):
        if not self.odom_received:
            return

        if self.state == WAITING:
            if self.started:
                self.state = ROTATING
                self.get_logger().info(
                    f'Starting path — {len(WAYPOINTS)} waypoints')
            return

        if self.state == DONE:
            self._publish_stop()
            return

        if self.state == PAUSING:
            if time.time() - self.pause_t >= STOP_DURATION:
                self.wp_idx += 1
                if self.wp_idx >= len(WAYPOINTS):
                    self.state = DONE
                    self._publish_stop()
                    self.get_logger().info(
                        '✓ Path complete — returned to origin')
                else:
                    self.state = ROTATING
                    self.get_logger().info(
                        f'→ Waypoint {self.wp_idx+1}/{len(WAYPOINTS)}: '
                        f'({WAYPOINTS[self.wp_idx][0]:.2f}, '
                        f'{WAYPOINTS[self.wp_idx][1]:.2f})')
            return

        # Current target waypoint
        wx, wy = WAYPOINTS[self.wp_idx]
        dx = wx - self.x
        dy = wy - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        # Check if we reached the waypoint
        if dist < WAYPOINT_TOLERANCE:
            self._publish_stop()
            self.state = PAUSING
            self.pause_t = time.time()
            self.get_logger().info(
                f'✓ Reached waypoint {self.wp_idx+1}: '
                f'x={self.x:.3f} y={self.y:.3f}  '
                f'(error={dist*100:.1f} cm)')
            return

        # Desired heading to waypoint
        desired_heading = math.atan2(dy, dx)
        heading_error   = angle_wrap(desired_heading - self.theta)

        twist = Twist()

        if self.state == ROTATING:
            # Rotate in place until heading is close enough
            if abs(heading_error) < HEADING_TOLERANCE:
                self.state = DRIVING
            else:
                twist.angular.z = max(
                    -MAX_ANGULAR_SPEED,
                    min(MAX_ANGULAR_SPEED,
                        KP_ANGULAR * heading_error))
                self.cmd_pub.publish(twist)
                return

        if self.state == DRIVING:
            # If heading drifts too far, go back to rotating
            if abs(heading_error) > 3 * HEADING_TOLERANCE:
                self.state = ROTATING
                return

            # Drive forward with proportional heading correction
            # Speed slows down near waypoint for smooth arrival
            speed = min(MAX_LINEAR_SPEED, KP_LINEAR * dist)
            twist.linear.x  = speed
            twist.angular.z = max(
                -MAX_ANGULAR_SPEED,
                min(MAX_ANGULAR_SPEED,
                    KP_ANGULAR * heading_error))
            self.cmd_pub.publish(twist)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ── Local mode: wait for Enter key ──────────────────────────────────────
    def _wait_for_keypress(self):
        """
        Instead of stdin (which has buffering issues),
        wait for a message on /path_start topic.
        Start the robot by running in another terminal:
            ros2 topic pub /path_start std_msgs/msg/Empty {} --once
        """
        # from std_msgs.msg import Empty
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

    # ── ZMQ mode: listen for start command from VM ──────────────────────────
    def _start_zmq_listener(self, port: int):
        try:
            import zmq
        except ImportError:
            self.get_logger().error(
                'zmq not installed: pip3 install pyzmq --break-system-packages')
            return

        def _listen():
            ctx = zmq.Context.instance()
            sock = ctx.socket(zmq.REP)
            sock.bind(f'tcp://0.0.0.0:{port}')
            self.get_logger().info(
                f'ZMQ listener bound on port {port} — waiting for start from VM')
            while rclpy.ok():
                try:
                    msg = sock.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.1)
                    continue
                if msg == 'START':
                    self.started = True
                    sock.send_string('OK')
                    self.get_logger().info('START received from VM — beginning path')
                elif msg == 'STATUS':
                    sock.send_string(f'{self.state}:{self.wp_idx}')
                elif msg == 'STOP':
                    sock.send_string('OK')
                    self.state = DONE
                    self._publish_stop()
                else:
                    sock.send_string('UNKNOWN')

        t = threading.Thread(target=_listen, daemon=True)
        t.start()


def main():
    parser = argparse.ArgumentParser(description='Path follower for robot1')
    parser.add_argument('--mode', choices=['local', 'zmq'], default='local',
                        help='local = start on Enter key; zmq = wait for VM trigger')
    parser.add_argument('--zmq-port', type=int, default=5560,
                        help='ZMQ REP port (zmq mode only, default 5560)')
    args = parser.parse_args()

    rclpy.init()
    node = PathFollowerNode(mode=args.mode, zmq_port=args.zmq_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_stop()
        node.get_logger().info('Stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
