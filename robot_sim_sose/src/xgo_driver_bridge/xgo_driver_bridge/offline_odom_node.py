import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class OfflineOdometryNode(Node):
    def __init__(self):
        super().__init__('xgo_offline_odom')

        self.input_twist_topic = self.declare_parameter(
            'input_twist_topic',
            '/xgo/applied_vel',
        ).value
        self.input_imu_topic = self.declare_parameter(
            'input_imu_topic',
            '/imu/data',
        ).value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        self.publish_rate_hz = max(float(self.declare_parameter('publish_rate_hz', 20.0).value), 1.0)
        self.command_timeout_sec = float(self.declare_parameter('command_timeout_sec', 1.5).value)
        self.max_integration_dt_sec = float(self.declare_parameter('max_integration_dt_sec', 0.25).value)
        self.linear_velocity_scale = float(self.declare_parameter('linear_velocity_scale', 1.0).value)
        self.angular_velocity_scale = float(self.declare_parameter('angular_velocity_scale', 1.0).value)
        self.initial_x = float(self.declare_parameter('initial_x', 0.0).value)
        self.initial_y = float(self.declare_parameter('initial_y', 0.0).value)
        self.initial_yaw = float(self.declare_parameter('initial_yaw', 0.0).value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        twist_qos = QoSProfile(depth=50)
        self.create_subscription(
            TwistStamped,
            self.input_twist_topic,
            self.twist_callback,
            twist_qos,
        )
        self.create_subscription(
            Imu,
            self.input_imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.latest_imu_wz = 0.0
        self.have_absolute_yaw = False

        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw_rad = self.initial_yaw

        self.last_update_time = None
        self.last_twist_time = None
        self.last_imu_time = None
        self.inputs_received = False
        self.last_log_time = self.get_clock().now()

        self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            'Offline odometry started: '
            f'{self.input_twist_topic} + {self.input_imu_topic} -> {self.odom_topic}.'
        )

    def twist_callback(self, msg: TwistStamped) -> None:
        self.current_vx = float(msg.twist.linear.x) * self.linear_velocity_scale
        self.current_vy = float(msg.twist.linear.y) * self.linear_velocity_scale
        self.current_wz = float(msg.twist.angular.z) * self.angular_velocity_scale
        self.last_twist_time = self.time_from_msg(msg.header.stamp)
        self.inputs_received = True

    def imu_callback(self, msg: Imu) -> None:
        stamp = self.time_from_msg(msg.header.stamp)
        self.latest_imu_wz = float(msg.angular_velocity.z)

        orientation_valid = False
        if len(msg.orientation_covariance) >= 1:
            orientation_valid = msg.orientation_covariance[0] >= 0.0
        if (
            math.isfinite(msg.orientation.w)
            and math.isfinite(msg.orientation.x)
            and math.isfinite(msg.orientation.y)
            and math.isfinite(msg.orientation.z)
            and abs(msg.orientation.w) + abs(msg.orientation.x) + abs(msg.orientation.y) + abs(msg.orientation.z) > 0.0
        ):
            orientation_valid = orientation_valid or any(
                abs(value) > 1e-9
                for value in (
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                )
            )

        if orientation_valid:
            self.yaw_rad = yaw_from_quaternion(msg.orientation)
            self.have_absolute_yaw = True

        self.last_imu_time = stamp
        self.inputs_received = True

    def timer_callback(self) -> None:
        if not self.inputs_received:
            return

        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return

        if self.last_update_time is None:
            self.last_update_time = now
            self.publish_odom(now)
            return

        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        dt = min(dt, self.max_integration_dt_sec)
        self.last_update_time = now

        if self.command_is_stale(now):
            vx = 0.0
            vy = 0.0
            wz = 0.0
        else:
            vx = self.current_vx
            vy = self.current_vy
            wz = self.current_wz

        if not self.have_absolute_yaw:
            fallback_wz = self.latest_imu_wz if math.isfinite(self.latest_imu_wz) else wz
            self.yaw_rad += fallback_wz * dt

        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt

        self.publish_odom(now, vx=vx, vy=vy, wz=wz)
        self.maybe_log_state(now, vx, wz)

    def command_is_stale(self, now) -> bool:
        if self.last_twist_time is None:
            return True
        age_sec = (now - self.last_twist_time).nanoseconds / 1e9
        return age_sec > self.command_timeout_sec

    def publish_odom(self, stamp, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
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
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
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

    def maybe_log_state(self, now, vx: float, wz: float) -> None:
        if (now - self.last_log_time) < Duration(seconds=5.0):
            return

        self.last_log_time = now
        self.get_logger().info(
            'odom replay pose='
            f'({self.x:+.2f}, {self.y:+.2f}, yaw={math.degrees(self.yaw_rad):+.1f} deg), '
            f'cmd=({vx:+.2f} m/s, wz={wz:+.2f} rad/s), '
            f'absolute_yaw={"yes" if self.have_absolute_yaw else "no"}'
        )

    @staticmethod
    def time_from_msg(stamp):
        return Time.from_msg(stamp)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
