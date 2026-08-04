#!/usr/bin/env python3
"""Report basic occupancy-map growth and map-to-odom correction jumps."""

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class SlamFailureMonitor(Node):
    def __init__(self):
        super().__init__('slam_failure_monitor')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 50)
        self.create_timer(2.0, self.report)

        self.last_map_stats = None
        self.last_map_to_odom = None
        self.max_tf_step = 0.0
        self.max_yaw_step = 0.0
        self.map_updates = 0
        self.tf_updates = 0

    def map_callback(self, msg):
        total = len(msg.data)
        if total == 0:
            return

        occupied = sum(1 for cell in msg.data if cell >= 65)
        unknown = sum(1 for cell in msg.data if cell < 0)
        known = total - unknown
        resolution = msg.info.resolution
        cell_area = resolution * resolution

        self.last_map_stats = {
            'occupied': occupied,
            'known': known,
            'unknown': unknown,
            'occupied_area': occupied * cell_area,
            'known_area': known * cell_area,
        }
        self.map_updates += 1

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if parent != 'map' or child != 'odom':
                continue

            translation = transform.transform.translation
            yaw = yaw_from_quaternion(transform.transform.rotation)
            current = (translation.x, translation.y, yaw)

            if self.last_map_to_odom is not None:
                last_x, last_y, last_yaw = self.last_map_to_odom
                step = math.hypot(translation.x - last_x, translation.y - last_y)
                yaw_step = abs(angle_delta(yaw, last_yaw))
                self.max_tf_step = max(self.max_tf_step, step)
                self.max_yaw_step = max(self.max_yaw_step, yaw_step)

            self.last_map_to_odom = current
            self.tf_updates += 1

    def report(self):
        if self.last_map_stats is None:
            self.get_logger().info('Waiting for /map from the mapper...')
            return

        stats = self.last_map_stats
        occupied_percent = 0.0
        if stats['known'] > 0:
            occupied_percent = 100.0 * stats['occupied'] / stats['known']

        if self.last_map_to_odom is None:
            tf_text = 'map->odom unavailable'
        else:
            x, y, yaw = self.last_map_to_odom
            tf_text = (
                f'map->odom=({x:+.3f}, {y:+.3f}, '
                f'yaw={math.degrees(yaw):+.1f} deg), '
                f'max correction step={self.max_tf_step:.3f} m, '
                f'max yaw step={math.degrees(self.max_yaw_step):.1f} deg'
            )

        self.get_logger().info(
            'map updates=%d, occupied=%d cells (%.2f m^2, %.1f%% of known), '
            'known area=%.2f m^2, %s'
            % (
                self.map_updates,
                stats['occupied'],
                stats['occupied_area'],
                occupied_percent,
                stats['known_area'],
                tf_text,
            )
        )


def main():
    rclpy.init()
    node = SlamFailureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
