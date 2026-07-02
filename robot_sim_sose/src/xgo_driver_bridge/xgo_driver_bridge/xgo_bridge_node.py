import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

try:
    from xgolib import XGO
except ImportError:  # pragma: no cover - only hit outside the robot container.
    XGO = None


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class XgoBridgeNode(Node):
    def __init__(self):
        super().__init__('xgo_driver_bridge')

        self.port = self.declare_parameter('port', '/dev/ttyS0').value
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.version = self.declare_parameter('version', 'xgomini').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 20.0).value)
        self.cmd_vel_timeout_sec = float(self.declare_parameter('cmd_vel_timeout_sec', 0.5).value)
        self.enable_motion = bool(self.declare_parameter('enable_motion', False).value)
        self.enable_wheel_mode = bool(self.declare_parameter('enable_wheel_mode', True).value)
        self.max_linear_x = float(self.declare_parameter('max_linear_x', 0.20).value)
        self.max_linear_y = float(self.declare_parameter('max_linear_y', 0.0).value)
        self.max_angular_z = float(self.declare_parameter('max_angular_z', 0.80).value)

        self.robot = self.connect_robot()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.yaw_pub = self.create_publisher(Float32, '/xgo/yaw_deg', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0
        self.last_wheel_error: Optional[str] = None

        if self.robot is not None and self.enable_motion and self.enable_wheel_mode:
            self.try_enable_wheel_mode()

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)
        self.get_logger().info(
            f'XGO bridge started on {self.port} at {self.baud} baud. '
            f'Motion enabled: {self.enable_motion}.'
        )

    def connect_robot(self):
        if XGO is None:
            self.get_logger().error('xgolib is not installed in this environment.')
            return None

        try:
            robot = XGO(port=self.port, baud=self.baud, version=self.version)
            self.get_logger().info(
                f'Connected to XGO on {self.port}: '
                f'lib={self.safe_call(robot.read_lib_version)}, '
                f'firmware={self.safe_call(robot.read_firmware)}'
            )
            return robot
        except Exception as exc:
            self.get_logger().error(f'Could not connect to XGO on {self.port}: {exc!r}')
            return None

    def try_enable_wheel_mode(self):
        try:
            self.robot.enable_wheel_control(True)
            self.get_logger().info('Enabled XGO wheel-control mode.')
        except Exception as exc:
            self.get_logger().warning(f'Could not enable wheel-control mode: {exc!r}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        if self.enable_motion:
            self.send_wheel_command(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = max((now - self.last_update_time).nanoseconds / 1e9, 1e-3)
        self.last_update_time = now

        if self.command_timed_out(now):
            self.last_cmd = Twist()
            if self.enable_motion:
                self.send_wheel_command(self.last_cmd)

        yaw_deg = self.read_float('read_yaw')
        if yaw_deg is not None:
            self.yaw_rad = math.radians(yaw_deg)
            yaw_msg = Float32()
            yaw_msg.data = float(yaw_deg)
            self.yaw_pub.publish(yaw_msg)

        self.integrate_commanded_odom(dt)
        self.publish_imu(now, yaw_deg)
        self.publish_battery(now)
        self.publish_odom(now)

    def command_timed_out(self, now) -> bool:
        return (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_vel_timeout_sec

    def integrate_commanded_odom(self, dt: float):
        vx = self.clamp(self.last_cmd.linear.x, -self.max_linear_x, self.max_linear_x)
        vy = self.clamp(self.last_cmd.linear.y, -self.max_linear_y, self.max_linear_y)
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt

    def send_wheel_command(self, msg: Twist):
        if self.robot is None:
            return

        vx = self.clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        vy = self.clamp(msg.linear.y, -self.max_linear_y, self.max_linear_y)
        wz = self.clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)
        command = [vx, vy, wz]

        try:
            self.robot.wheel_control(command)
            self.last_wheel_error = None
        except Exception as exc:
            error = repr(exc)
            if error != self.last_wheel_error:
                self.last_wheel_error = error
                self.get_logger().error(
                    f'wheel_control({command}) failed: {error}. '
                    'Motion is not disabled automatically; run robot_stack.sh stop if needed.'
                )

    def publish_imu(self, stamp, yaw_deg: Optional[float]):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp.to_msg()
        imu_msg.header.frame_id = self.base_frame_id

        if yaw_deg is not None:
            qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.orientation_covariance[8] = 0.05
        else:
            imu_msg.orientation_covariance[0] = -1.0

        imu = self.safe_call(self.robot.read_imu) if self.robot is not None else None
        if isinstance(imu, list) and len(imu) >= 6:
            imu_msg.linear_acceleration.x = float(imu[0])
            imu_msg.linear_acceleration.y = float(imu[1])
            imu_msg.linear_acceleration.z = float(imu[2])
            imu_msg.angular_velocity.x = float(imu[3])
            imu_msg.angular_velocity.y = float(imu[4])
            imu_msg.angular_velocity.z = float(imu[5])

        self.imu_pub.publish(imu_msg)

    def publish_battery(self, stamp):
        battery = self.read_float('read_battery')
        if battery is None:
            return

        msg = BatteryState()
        msg.header.stamp = stamp.to_msg()
        msg.voltage = float(battery)
        msg.present = True
        self.battery_pub.publish(msg)

    def publish_odom(self, stamp):
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
        odom.twist.twist = self.last_cmd
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

    def read_float(self, method_name: str) -> Optional[float]:
        if self.robot is None:
            return None
        value = self.safe_call(getattr(self.robot, method_name))
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def safe_call(callback):
        try:
            return callback()
        except Exception:
            return None

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return min(max(float(value), lower), upper)


def main(args=None):
    rclpy.init(args=args)
    node = XgoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
