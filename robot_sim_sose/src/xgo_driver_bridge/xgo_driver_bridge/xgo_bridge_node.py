import math
from importlib.metadata import version as package_version
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

try:
    import xgolib

    XGO = xgolib.XGO
    try:
        XGOLIB_VERSION = package_version('xgolib')
    except Exception:
        XGOLIB_VERSION = getattr(xgolib, '__version__', 'unknown')
except ImportError:  # pragma: no cover - only hit outside the robot container.
    XGO = None
    XGOLIB_VERSION = 'not installed'


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class XgoBridgeNode(Node):
    """Bridge the XGO Mini2 controller to the ROS 2 hardware interface."""

    def __init__(self):
        super().__init__('xgo_driver_bridge')

        self.port = self.declare_parameter('port', '/dev/ttyAMA0').value
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.version = self.declare_parameter('version', 'xgomini').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.publish_legacy_imu_topic = bool(
            self.declare_parameter('publish_legacy_imu_topic', True).value
        )
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        self.publish_rate_hz = max(
            float(self.declare_parameter('publish_rate_hz', 20.0).value),
            1.0,
        )
        self.battery_publish_rate_hz = max(
            float(self.declare_parameter('battery_publish_rate_hz', 1.0).value),
            0.1,
        )
        self.imu_read_mode = str(
            self.declare_parameter(
                'imu_read_mode',
                'orientation_registers',
            ).value
        ).strip()
        if self.imu_read_mode not in {'orientation_registers', 'combined'}:
            raise ValueError(
                'imu_read_mode must be orientation_registers or combined, '
                f'not {self.imu_read_mode!r}.'
            )
        self.imu_relative_yaw = bool(
            self.declare_parameter('imu_relative_yaw', True).value
        )
        self.cmd_vel_timeout_sec = max(
            float(self.declare_parameter('cmd_vel_timeout_sec', 0.5).value),
            0.05,
        )
        self.max_integration_dt_sec = max(
            float(self.declare_parameter('max_integration_dt_sec', 0.25).value),
            0.01,
        )
        self.enable_motion = bool(self.declare_parameter('enable_motion', False).value)

        # ROS velocities are SI units. The Mini2 SDK accepts approximately cm/s
        # for X/Y gait speed and deg/s for yaw speed.
        self.max_linear_x = max(
            float(self.declare_parameter('max_linear_x', 0.20).value),
            0.0,
        )
        self.max_linear_y = max(
            float(self.declare_parameter('max_linear_y', 0.0).value),
            0.0,
        )
        self.max_angular_z = max(
            float(self.declare_parameter('max_angular_z', 0.80).value),
            0.0,
        )
        self.linear_command_units_per_mps = max(
            float(
                self.declare_parameter(
                    'linear_command_units_per_mps',
                    100.0,
                ).value
            ),
            0.0,
        )
        self.sdk_max_vx = max(
            float(self.declare_parameter('sdk_max_vx', 25.0).value),
            0.0,
        )
        self.sdk_max_vy = max(
            float(self.declare_parameter('sdk_max_vy', 18.0).value),
            0.0,
        )
        self.sdk_max_vyaw_deg = max(
            float(self.declare_parameter('sdk_max_vyaw_deg', 100.0).value),
            0.0,
        )

        self.robot = self.connect_robot()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_pub = self.create_publisher(
            Imu,
            self.imu_topic,
            qos_profile_sensor_data,
        )
        self.legacy_imu_pub = None
        if self.publish_legacy_imu_topic and self.imu_topic != '/imu':
            self.legacy_imu_pub = self.create_publisher(
                Imu,
                '/imu',
                qos_profile_sensor_data,
            )
        self.battery_pub = self.create_publisher(
            BatteryState,
            '/battery_state',
            10,
        )
        self.yaw_pub = self.create_publisher(Float32, '/xgo/yaw_deg', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.applied_velocity_pub = self.create_publisher(
            TwistStamped,
            '/xgo/applied_vel',
            10,
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.motion_active = False
        self.last_cmd_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0
        self.imu_yaw_reference = None
        self.last_orientation_raw_yaw = None
        self.unwrapped_orientation_yaw = None
        self.orientation_sample_count = 0
        self.last_update_time = self.get_clock().now()
        self.last_battery_time = None
        self.last_sensor_warning_time = None
        self.last_motion_error: Optional[str] = None
        self.hardware_shutdown = False

        self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            f'XGO bridge started: SDK={XGOLIB_VERSION}, port={self.port}, '
            f'baud={self.baud}, model={self.version}, '
            f'motion_enabled={self.enable_motion}, imu_topic={self.imu_topic}, '
            f'imu_read_mode={self.imu_read_mode}.'
        )
        if self.imu_read_mode == 'orientation_registers':
            self.get_logger().info(
                'Using the XGO roll/pitch/yaw registers for orientation. '
                'Yaw is zero-referenced at startup and unwrapped between '
                'samples. Angular velocity and linear acceleration are marked '
                'unavailable in sensor_msgs/Imu.'
            )
        if not self.enable_motion:
            self.get_logger().info(
                'Motion is disabled. /cmd_vel will not be sent to the robot '
                'or integrated into odometry.'
            )

    def connect_robot(self):
        if XGO is None:
            self.get_logger().error(
                'The official xgolib SDK is not installed in this environment.'
            )
            return None

        try:
            robot = XGO(
                version=self.version,
                port=self.port,
                baud=self.baud,
            )
            firmware = self.call_sdk(robot.read_firmware)
            library_version = self.call_sdk(robot.read_lib_version)
            self.get_logger().info(
                f'Connected to XGO hardware: firmware={firmware}, '
                f'controller_library={library_version}.'
            )
            return robot
        except Exception as exc:
            self.get_logger().error(
                f'Could not connect to XGO on {self.port}: {exc!r}. '
                'Check that /dev/ttyAMA0 is mapped into the container and that '
                'no other process owns the serial port.'
            )
            return None

    def cmd_vel_callback(self, msg: Twist) -> None:
        if not self.enable_motion:
            return
        if self.robot is None:
            self.maybe_warn('Cannot apply /cmd_vel: XGO hardware is disconnected.')
            return

        vx = self.clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        vy = self.clamp(msg.linear.y, -self.max_linear_y, self.max_linear_y)
        wz = self.clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)

        if not self.send_mini2_command(vx, vy, wz):
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
            self.motion_active = False
            return

        self.current_vx = vx
        self.current_vy = vy
        self.current_wz = wz
        self.motion_active = any(abs(value) > 1e-9 for value in (vx, vy, wz))
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0:
            return
        dt = min(dt, self.max_integration_dt_sec)

        if self.motion_active and self.command_is_stale(now):
            self.stop_motion('cmd_vel timeout')

        imu_values = self.read_imu_sample(now)
        if imu_values is not None:
            self.publish_imu(now, imu_values)

        self.publish_applied_velocity(now)
        self.integrate_commanded_odom(dt)
        self.publish_odom(now)

        if self.battery_is_due(now):
            self.publish_battery(now)

    def command_is_stale(self, now) -> bool:
        if self.last_cmd_time is None:
            return True
        age_sec = (now - self.last_cmd_time).nanoseconds / 1e9
        return age_sec > self.cmd_vel_timeout_sec

    def publish_applied_velocity(self, stamp) -> None:
        msg = TwistStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.twist.linear.x = self.current_vx
        msg.twist.linear.y = self.current_vy
        msg.twist.angular.z = self.current_wz
        self.applied_velocity_pub.publish(msg)

    def send_mini2_command(self, vx: float, vy: float, wz: float) -> bool:
        sdk_vx = int(
            round(
                self.clamp(
                    vx * self.linear_command_units_per_mps,
                    -self.sdk_max_vx,
                    self.sdk_max_vx,
                )
            )
        )
        sdk_vy = int(
            round(
                self.clamp(
                    vy * self.linear_command_units_per_mps,
                    -self.sdk_max_vy,
                    self.sdk_max_vy,
                )
            )
        )
        sdk_vyaw = int(
            round(
                self.clamp(
                    math.degrees(wz),
                    -self.sdk_max_vyaw_deg,
                    self.sdk_max_vyaw_deg,
                )
            )
        )

        try:
            if sdk_vx == 0 and sdk_vy == 0 and sdk_vyaw == 0:
                self.robot.stop()
            else:
                # Send all three axes, including zeros, so an old axis command
                # cannot remain active in the XGO controller.
                self.robot.move_x(sdk_vx)
                self.robot.move_y(sdk_vy)
                self.robot.turn(sdk_vyaw)
            self.last_motion_error = None
            return True
        except Exception as exc:
            error = repr(exc)
            if error != self.last_motion_error:
                self.last_motion_error = error
                self.get_logger().error(
                    'XGO Mini2 gait command failed: '
                    f'ROS=({vx:+.3f} m/s, {vy:+.3f} m/s, {wz:+.3f} rad/s), '
                    f'SDK=({sdk_vx}, {sdk_vy}, {sdk_vyaw}), error={error}.'
                )
            self.safe_hardware_stop()
            return False

    def stop_motion(self, reason: str) -> None:
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        was_active = self.motion_active
        self.motion_active = False
        self.safe_hardware_stop()
        if was_active:
            self.get_logger().warning(f'Stopped XGO motion: {reason}.')

    def safe_hardware_stop(self) -> None:
        if self.robot is None or not self.enable_motion:
            return
        try:
            self.robot.stop()
        except Exception as exc:
            self.get_logger().error(f'Failed to send XGO stop command: {exc!r}.')

    def read_imu_sample(self, stamp) -> Optional[Sequence[float]]:
        if self.robot is None:
            return None

        if self.imu_read_mode == 'orientation_registers':
            return self.read_orientation_register_sample()

        try:
            values = self.robot.read_imu()
        except Exception as exc:
            self.maybe_warn(f'XGO IMU read failed: {exc!r}.')
            return None

        if not isinstance(values, (list, tuple)) or len(values) < 9:
            self.maybe_warn(
                f'XGO IMU returned {len(values) if values is not None else 0} '
                'values; expected acceleration, gyro, and roll/pitch/yaw.'
            )
            return None
        if not all(math.isfinite(float(value)) for value in values[:9]):
            self.maybe_warn('XGO IMU returned a non-finite value.')
            return None
        return values

    def read_orientation_register_sample(self) -> Optional[Sequence[float]]:
        """Read the controller Euler-angle registers validated on firmware M-5.1.1."""
        try:
            roll_deg = float(self.robot.read_roll())
            pitch_deg = float(self.robot.read_pitch())
            yaw_deg = float(self.robot.read_yaw())
        except Exception as exc:
            self.maybe_warn(f'XGO orientation-register read failed: {exc!r}.')
            return None

        values = (roll_deg, pitch_deg, yaw_deg)
        if not all(math.isfinite(value) for value in values):
            self.maybe_warn('XGO orientation register returned a non-finite value.')
            return None
        if abs(roll_deg) > 180.0 or abs(pitch_deg) > 180.0:
            self.maybe_warn(
                'XGO orientation register returned an invalid roll or pitch angle.'
            )
            return None

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        raw_yaw = math.radians(yaw_deg)
        if self.last_orientation_raw_yaw is None:
            self.unwrapped_orientation_yaw = raw_yaw
        else:
            self.unwrapped_orientation_yaw += math.atan2(
                math.sin(raw_yaw - self.last_orientation_raw_yaw),
                math.cos(raw_yaw - self.last_orientation_raw_yaw),
            )
        self.last_orientation_raw_yaw = raw_yaw
        if self.imu_yaw_reference is None:
            self.imu_yaw_reference = self.unwrapped_orientation_yaw
        yaw = (
            self.unwrapped_orientation_yaw - self.imu_yaw_reference
            if self.imu_relative_yaw
            else self.unwrapped_orientation_yaw
        )

        self.orientation_sample_count += 1
        if self.orientation_sample_count == 1:
            self.get_logger().info(
                'First valid XGO orientation-register sample: '
                f'roll={roll_deg:+.2f} deg, pitch={pitch_deg:+.2f} deg, '
                f'raw_yaw={yaw_deg:+.2f} deg; relative yaw is now zero.'
            )

        # The three controller registers contain only Euler angles. Zeros are
        # placeholders; covariance[0] == -1 below tells ROS consumers that
        # angular velocity and linear acceleration are unavailable.
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, roll, pitch, yaw

    def publish_imu(self, stamp, values: Sequence[float]) -> None:
        ax, ay, az = (float(value) for value in values[0:3])
        # The official Mini2 SDK decodes its gyroscope values as degrees/s.
        gx, gy, gz = (math.radians(float(value)) for value in values[3:6])
        roll, pitch, yaw = (float(value) for value in values[6:9])
        self.yaw_rad = yaw

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.05,
        ]
        if self.imu_read_mode == 'orientation_registers':
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
        else:
            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02,
            ]
            msg.linear_acceleration_covariance = [
                0.10, 0.0, 0.0,
                0.0, 0.10, 0.0,
                0.0, 0.0, 0.10,
            ]

        self.imu_pub.publish(msg)
        if self.legacy_imu_pub is not None:
            self.legacy_imu_pub.publish(msg)

        yaw_msg = Float32()
        yaw_msg.data = math.degrees(yaw)
        self.yaw_pub.publish(yaw_msg)

    def integrate_commanded_odom(self, dt: float) -> None:
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.x += (
            self.current_vx * cos_yaw - self.current_vy * sin_yaw
        ) * dt
        self.y += (
            self.current_vx * sin_yaw + self.current_vy * cos_yaw
        ) * dt

    def publish_odom(self, stamp) -> None:
        # Keep the SLAM odometry transform planar. Roll and pitch are available
        # on the IMU topic but must not tilt the 2D odom -> base_link transform.
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw_rad)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.current_vx
        odom.twist.twist.linear.y = self.current_vy
        odom.twist.twist.angular.z = self.current_wz

        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 1.0e6
        odom.pose.covariance[21] = 1.0e6
        odom.pose.covariance[28] = 1.0e6
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.10
        odom.twist.covariance[7] = 0.10
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def battery_is_due(self, now) -> bool:
        if self.last_battery_time is None:
            return True
        interval = Duration(seconds=1.0 / self.battery_publish_rate_hz)
        return (now - self.last_battery_time) >= interval

    def publish_battery(self, stamp) -> None:
        self.last_battery_time = stamp
        if self.robot is None:
            return

        try:
            battery_percent = float(self.robot.read_battery())
        except Exception as exc:
            self.maybe_warn(f'XGO battery read failed: {exc!r}.')
            return

        if not math.isfinite(battery_percent):
            self.maybe_warn('XGO battery read returned a non-finite value.')
            return

        msg = BatteryState()
        msg.header.stamp = stamp.to_msg()
        # read_battery() exposes percentage only. BatteryState specifies NaN
        # for unavailable measurements; leaving these fields at their Python
        # default of 0.0 would incorrectly imply measured zero values.
        msg.voltage = math.nan
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = self.clamp(battery_percent / 100.0, 0.0, 1.0)
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.battery_pub.publish(msg)

    def maybe_warn(self, message: str) -> None:
        now = self.get_clock().now()
        if (
            self.last_sensor_warning_time is not None
            and (now - self.last_sensor_warning_time) < Duration(seconds=5.0)
        ):
            return
        self.last_sensor_warning_time = now
        self.get_logger().warning(message)

    @staticmethod
    def call_sdk(callback):
        try:
            return callback()
        except Exception as exc:
            return f'unavailable ({exc!r})'

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return min(max(float(value), lower), upper)

    def shutdown_hardware(self) -> None:
        if self.hardware_shutdown:
            return
        self.hardware_shutdown = True
        if self.enable_motion:
            self.safe_hardware_stop()
        serial_port = getattr(self.robot, 'ser', None)
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = XgoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
