#!/usr/bin/env python3
"""
mpu6050_node.py — MPU6050 IMU reader for RPi4
----------------------------------------------
Reads the MPU6050 directly from the RPi4's I2C bus (GPIO2/GPIO3)
and publishes sensor_msgs/Imu to /imu/data.

No ESP32 involved — the MPU6050 is wired directly to the RPi4:
  VCC → RPi4 Pin 1  (3.3V)
  GND → RPi4 Pin 6  (GND)
  SDA → RPi4 Pin 3  (GPIO2)
  SCL → RPi4 Pin 5  (GPIO3)
  AD0 → RPi4 Pin 9  (GND)  ← sets I2C address to 0x68

Verify the sensor is visible before running this node:
  sudo apt install i2c-tools
  i2cdetect -y 1          ← should show "68" in the grid

Install dependency:
  pip3 install smbus2
"""

import math
import time

import rclpy
import smbus2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# ── MPU6050 register addresses ───────────────────────────────────────────────
PWR_MGMT_1      = 0x6B
ACCEL_XOUT_H    = 0x3B
SMPLRT_DIV      = 0x19
CONFIG_REG      = 0x1A
GYRO_CONFIG     = 0x1B
ACCEL_CONFIG    = 0x1C

# ── Sensitivity scales ───────────────────────────────────────────────────────
# Accelerometer: ±2g range → 16384 LSB/g
ACCEL_SCALE = 16384.0
GRAVITY     = 9.80665  # m/s²

# Gyroscope: ±250°/s range → 131 LSB/(°/s)
GYRO_SCALE  = 131.0


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 1)          # /dev/i2c-1 on RPi4
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_samples', 100)

        bus_num = int(self.get_parameter('i2c_bus').value)
        addr = int(self.get_parameter('i2c_address').value)
        rate_hz = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.calibration_samples = int(
            self.get_parameter('calibration_samples').value
        )

        if rate_hz <= 0.0:
            raise ValueError('publish_rate must be > 0 Hz')

        if self.calibration_samples <= 0:
            raise ValueError('calibration_samples must be > 0')

        # Gyro biases in rad/s.  For your EKF yaw, gyro_z_bias is the important
        # one, but correcting all three axes makes the IMU message consistent.
        self.gyro_x_bias = 0.0
        self.gyro_y_bias = 0.0
        self.gyro_z_bias = 0.0

        # ── Open I2C bus ─────────────────────────────────────────────────────
        try:
            self.bus = smbus2.SMBus(bus_num)
            self.addr = addr
        except Exception as e:
            self.get_logger().error(
                f'Cannot open I2C bus {bus_num}: {e}\n'
                f'Enable I2C with: sudo raspi-config → Interface Options → I2C'
            )
            raise

        # ── Initialise MPU6050 ───────────────────────────────────────────────
        self.get_logger().info('Initialising MPU6050...')
        self._init_mpu6050()

        # ── Publisher (BEST_EFFORT matches common robot_localization configs) ─
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.imu_pub = self.create_publisher(Imu, '/imu/data', best_effort_qos)

        # ── Timer ────────────────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.publish_imu)

        self.get_logger().info(
            f'MPU6050 node started — publishing /imu/data at {rate_hz:.1f} Hz'
        )

    def _init_mpu6050(self):
        """Wake MPU6050, configure ranges, and estimate gyro bias."""
        # Wake up (clear sleep bit in PWR_MGMT_1)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.10)  # give the sensor time to wake up cleanly

        # Sample rate divider: SMPLRT_DIV = 0 → 1 kHz base / 1 = 1000 Hz
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x00)

        # Internal MPU6050 DLPF.
        # 0x02 is approximately 94 Hz gyro bandwidth. This is the same setting
        # as your original node. There is no extra software low-pass filter here.
        self.bus.write_byte_data(self.addr, CONFIG_REG, 0x02)

        # Gyroscope: ±250°/s (GYRO_CONFIG = 0x00)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)

        # Accelerometer: ±2g (ACCEL_CONFIG = 0x00)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)

        # ── Gyro bias calibration ────────────────────────────────────────────
        # Keep the robot stationary while this runs.
        self.get_logger().info(
            f'Calibrating gyro bias from {self.calibration_samples} samples — '
            f'keep robot stationary...'
        )

        gx_samples = []
        gy_samples = []
        gz_samples = []

        for _ in range(self.calibration_samples):
            _, _, _, gx_raw, gy_raw, gz_raw = self._read_all()
            gx_samples.append(math.radians(gx_raw / GYRO_SCALE))
            gy_samples.append(math.radians(gy_raw / GYRO_SCALE))
            gz_samples.append(math.radians(gz_raw / GYRO_SCALE))
            time.sleep(0.01)

        self.gyro_x_bias = sum(gx_samples) / len(gx_samples)
        self.gyro_y_bias = sum(gy_samples) / len(gy_samples)
        self.gyro_z_bias = sum(gz_samples) / len(gz_samples)

        self.get_logger().info(
            'Gyro bias [rad/s]: '
            f'x={self.gyro_x_bias:.5f}, '
            f'y={self.gyro_y_bias:.5f}, '
            f'z={self.gyro_z_bias:.5f} '
            f'(z={math.degrees(self.gyro_z_bias):.3f} deg/s)'
        )

        self.get_logger().info('MPU6050 initialised: ±2g accel, ±250°/s gyro')

    def _read_all(self):
        """
        Read 14 bytes starting at ACCEL_XOUT_H in a single burst read.
        Returns (ax, ay, az, gx, gy, gz) as raw signed integers.
        """
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)

        def to_signed(h, l):
            val = (h << 8) | l
            return val - 65536 if val >= 32768 else val

        ax = to_signed(data[0],  data[1])
        ay = to_signed(data[2],  data[3])
        az = to_signed(data[4],  data[5])
        # data[6], data[7] = temperature (not used)
        gx = to_signed(data[8],  data[9])
        gy = to_signed(data[10], data[11])
        gz = to_signed(data[12], data[13])

        return ax, ay, az, gx, gy, gz

    def publish_imu(self):
        """Read IMU and publish sensor_msgs/Imu."""
        try:
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self._read_all()
        except Exception as e:
            self.get_logger().warn(f'MPU6050 read error: {e}')
            return

        # Convert acceleration raw values to SI units [m/s²]
        ax = (ax_raw / ACCEL_SCALE) * GRAVITY
        ay = (ay_raw / ACCEL_SCALE) * GRAVITY
        az = (az_raw / ACCEL_SCALE) * GRAVITY

        # Convert gyro raw values to SI units [rad/s], then subtract bias ONCE.
        gx = math.radians(gx_raw / GYRO_SCALE) - self.gyro_x_bias
        gy = math.radians(gy_raw / GYRO_SCALE) - self.gyro_y_bias
        gz = math.radians(gz_raw / GYRO_SCALE) - self.gyro_z_bias

        # ── Build Imu message ────────────────────────────────────────────────
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation is not computed here.
        # covariance[0] = -1 tells consumers that orientation is unavailable.
        msg.orientation_covariance[0] = -1.0

        # Angular velocity [rad/s]
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        # Linear acceleration [m/s²]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(msg)

    def destroy_node(self):
        """Close the I2C bus cleanly on shutdown."""
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
