#!/usr/bin/env python3
"""
mpu6050_node.py — MPU6050 IMU reader for RPi4
----------------------------------------------
Reads the MPU6050 directly from the RPi4's I2C bus (GPIO2/GPIO3)
and publishes sensor_msgs/Imu to /imu/data at 50 Hz.

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
import smbus2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# ── MPU6050 register addresses ───────────────────────────────────────────────
MPU6050_ADDR    = 0x68   # AD0 tied to GND
PWR_MGMT_1      = 0x6B
ACCEL_XOUT_H    = 0x3B
GYRO_XOUT_H     = 0x43
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

        self.declare_parameter('i2c_bus',      1)       # /dev/i2c-1 on RPi4
        self.declare_parameter('i2c_address',  0x68)
        self.declare_parameter('publish_rate', 50.0)    # Hz
        self.declare_parameter('frame_id',     'imu_link')

        bus_num  = self.get_parameter('i2c_bus').value
        addr     = self.get_parameter('i2c_address').value
        rate_hz  = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # ── Open I2C bus ─────────────────────────────────────────────────────
        try:
            self.bus  = smbus2.SMBus(bus_num)
            self.addr = addr
        except Exception as e:
            self.get_logger().error(
                f'Cannot open I2C bus {bus_num}: {e}\n'
                f'Enable I2C with: sudo raspi-config → Interface Options → I2C')
            raise

        # ── Initialise MPU6050 ───────────────────────────────────────────────
        self.get_logger().info('Initialising MPU6050...')
        self.gyro_z_bias = 0.0   # will be set during init
        self._init_mpu6050()

        # ── Publisher (BEST_EFFORT matches EKF subscription) ─────────────────
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.imu_pub = self.create_publisher(Imu, '/imu/data', best_effort_qos)

        # ── Timer ────────────────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.publish_imu)

        self.get_logger().info(
            f'MPU6050 node started — publishing /imu/data at {rate_hz:.0f} Hz')

    def _init_mpu6050(self):
        """Wake MPU6050 and configure ranges."""
        # Wake up (clear sleep bit in PWR_MGMT_1)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)

        # Sample rate divider: SMPLRT_DIV = 0 → 1 kHz base / 1 = 1000 Hz
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x00)

        # DLPF: low-pass filter ~94 Hz bandwidth (config = 2)
        self.bus.write_byte_data(self.addr, CONFIG_REG, 0x02)

        # Gyroscope: ±250°/s (GYRO_CONFIG = 0x00)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)

        # Accelerometer: ±2g (ACCEL_CONFIG = 0x00)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)

        # ── Gyro bias calibration ─────────────────────────────────────────
        # Collect 100 samples while stationary to measure Z-axis bias
        self.get_logger().info('Calibrating gyro bias — keep robot stationary...')
        import time
        gz_samples = []
        for _ in range(100):
            _, _, _, gx_r, gy_r, gz_r = self._read_all()
            gz_samples.append(gz_r / GYRO_SCALE)  # deg/s
            time.sleep(0.01)
        self.gyro_z_bias = math.radians(sum(gz_samples) / len(gz_samples))
        self.get_logger().info(
            f'Gyro Z bias: {self.gyro_z_bias:.5f} rad/s '
            f'({math.degrees(self.gyro_z_bias):.3f} deg/s) — will be subtracted')

        self.get_logger().info('MPU6050 initialised: ±2g accel, ±250°/s gyro')

    def _read_word_signed(self, reg: int) -> int:
        """Read a 16-bit signed value from two consecutive registers."""
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val  = (high << 8) | low
        if val >= 32768:
            val -= 65536
        return val

    def _read_all(self):
        """
        Read 14 bytes starting at ACCEL_XOUT_H in a single burst read.
        Returns (ax, ay, az, temp, gx, gy, gz) as raw integers.
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

        # Convert raw to SI units
        # Acceleration: raw / scale * gravity  [m/s²]
        ax = (ax_raw / ACCEL_SCALE) * GRAVITY
        ay = (ay_raw / ACCEL_SCALE) * GRAVITY
        az = (az_raw / ACCEL_SCALE) * GRAVITY

        # Angular velocity: raw / scale → deg/s → rad/s
        gx = math.radians(gx_raw / GYRO_SCALE)
        gy = math.radians(gy_raw / GYRO_SCALE)
        gz = math.radians(gz_raw / GYRO_SCALE)

        # ── Build Imu message ─────────────────────────────────────────────
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation: not computed here — EKF will integrate gyro data
        # Set covariance[0] = -1 to indicate orientation is not provided
        msg.orientation_covariance[0] = -1.0

        # Angular velocity [rad/s] — subtract calibrated bias from Z axis
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz - self.gyro_z_bias  # Subtract Z bias
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
