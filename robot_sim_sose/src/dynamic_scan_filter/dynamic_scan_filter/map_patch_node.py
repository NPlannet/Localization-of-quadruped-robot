# -*- coding: utf-8 -*-
"""
Created on Tue Jul  7 14:53:33 2026

@author: Lucas
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

FREE = 0
UNKNOWN = -1
OCCUPIED = 100


@dataclass
class RemovedObjectPatch:
    marker_id: int
    frame_id: str
    x: float
    y: float
    radius: float
    reported_at_sec: float
    cells: Optional[List[Tuple[float, float]]] = None


class MapPatchNode(Node):
    def __init__(self):
        super().__init__('map_patch_node')
        
        self.live_cluster_topic = self.declare_parameter(
        'live_cluster_topic', '/dynamic_scan_filter/cluster_markers'
        ).value
        self.reappear_margin = float(
            self.declare_parameter('reappear_margin', 0.10).value
        )
        self.live_cluster_max_age_sec = float(
            self.declare_parameter('live_cluster_max_age_sec', 1.0).value
        )
        self.live_cluster_sub = self.create_subscription(
            MarkerArray, self.live_cluster_topic, self.live_cluster_callback, 10
        )

        
        self.live_cluster_points: List[Tuple[str, float, float, float]] = []

        # Robot-Frame fuer die Naehe-Pruefung ("nicht allzu weit entfernt").
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.removal_proximity_threshold = float(
            self.declare_parameter('removal_proximity_threshold', 3.0).value
        )
        # Toleranz: welcher Anteil der Objekt-Zellen darf trotzdem noch als
        # belegt gemeldet sein und das Objekt gilt weiterhin als "komplett fehlt"
        self.object_missing_tolerance = float(
            self.declare_parameter('object_missing_tolerance', 0.1).value
        )
        # Suchradius um die gemeldete Centroid-Position, um die naechste
        # belegte Zelle als Flood-Fill-Startpunkt zu finden.
        self.blob_search_margin = float(
            self.declare_parameter('blob_search_margin', 1.0).value
        )
        # Sicherheitsdeckel gegen ausufernden Flood-Fill (z.B. wenn die
        # Centroid-Position versehentlich auf einer Wand statt einem
        # einzelnen Objekt landet).
        self.max_object_cells = int(
            self.declare_parameter('max_object_cells', 4000).value
        )

        self.debug_blob_pub = self.create_publisher(MarkerArray, '/map_patch_node/debug_blobs', 10)
        self.gap_bridge_cells = int(
            self.declare_parameter('gap_bridge_cells', 2).value
        )
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.patched_map_topic = self.declare_parameter('patched_map_topic', '/map_patched').value
        self.removed_object_topic = self.declare_parameter(
            'removed_object_topic',
            '/dynamic_scan_filter/removed_static_objects',
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.patch_retention_sec = float(
            self.declare_parameter('patch_retention_sec', 300.0).value
        )
        self.transform_timeout_sec = float(
            self.declare_parameter('transform_timeout_sec', 0.2).value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.patched_map_topic, map_qos)
        self.removed_object_sub = self.create_subscription(
            Marker, self.removed_object_topic, self.removed_object_callback, 10
        )

        self.patches: List[RemovedObjectPatch] = []

        self.get_logger().info(
            f'map_patch_node gestartet: {self.map_topic} + {self.removed_object_topic} '
            f'-> {self.patched_map_topic}'
        )

    def removed_object_callback(self, marker: Marker) -> None:
        self.prune_expired_patches()
        radius = max(marker.scale.x, marker.scale.y) / 2.0
        patch = RemovedObjectPatch(
            marker_id=marker.id,
            frame_id=marker.header.frame_id,
            x=marker.pose.position.x,
            y=marker.pose.position.y,
            radius=radius,
            reported_at_sec=self.get_clock().now().nanoseconds / 1e9,
        )
        # Falls fuer dieselbe track_id (marker.id) bereits ein Patch existiert,
        # ersetzen statt duplizieren.
        self.patches = [p for p in self.patches if p.marker_id != patch.marker_id]
        self.patches.append(patch)
        self.get_logger().info(
            f'Neues Loesch-Areal registriert: id={patch.marker_id} '
            f'r={patch.radius:.2f}m in "{patch.frame_id}" -> werde bei naechstem '
            f'/map-Update angewendet.'
        )

    def live_cluster_callback(self, msg: MarkerArray) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        points: List[Tuple[str, float, float, float]] = []
        for marker in msg.markers:
            if marker.ns != 'cluster_centroids':
                continue
            points.append((
                marker.header.frame_id,
                marker.pose.position.x,
                marker.pose.position.y,
                now_sec,
            ))
        self.live_cluster_points = points
        
    def prune_expired_patches(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.patches = [
            p for p in self.patches
            if now_sec - p.reported_at_sec <= self.patch_retention_sec
        ]

    def map_callback(self, map_msg: OccupancyGrid) -> None:
        self.prune_expired_patches()

        self.get_logger().info(
            f'map_callback: {len(self.patches)} aktive Patch(es) zu pruefen.'
        )

        if not self.patches:
            self.map_pub.publish(map_msg)
            return

        patched = OccupancyGrid()
        patched.header = map_msg.header
        patched.header.frame_id = self.map_frame
        patched.info = map_msg.info
        patched.data = list(map_msg.data)

        robot_xy = self.get_robot_position(map_msg.header.stamp)
        applied = 0
        still_valid_patches: List[RemovedObjectPatch] = []

        for patch in self.patches:
            try:
                map_xy = self.transform_to_map(patch, map_msg.header.stamp)
                if map_xy is None:
                    still_valid_patches.append(patch)
                    continue
        
                if self.object_reappeared(patch, map_xy):
                    self.get_logger().info(
                        f'Loesch-Areal id={patch.marker_id} verworfen: '
                        f'Objekt wieder sichtbar (Live-Cluster erkannt).'
                    )
                    continue
        
                # 1) Objekt-Zellen einmalig auflösen (ganze zusammenhängende
                #    Blob-Form statt Kreis-Schaetzung), sobald noch nicht geschehen.
                if patch.cells is None:
                    patch.cells = self.resolve_object_cells(patched, map_xy[0], map_xy[1], patch.radius)
                    if patch.cells is None:
                        self.get_logger().info(
                            f'Patch id={patch.marker_id}: keine belegten Zellen bei map_xy=({map_xy[0]:.2f}, {map_xy[1]:.2f}) '
                            f'r={patch.radius:.2f} gefunden, warte weiter.'
                        )
                        still_valid_patches.append(patch)
                        continue
        
                # 2) Reichweiten-Check: ALLE Zellen des Objekts muessen innerhalb
                #    der Reichweite um den Roboter liegen (alles-oder-nichts).
                if robot_xy is None:
                    still_valid_patches.append(patch)
                    continue
                if not self.object_fully_in_range(patch.cells, robot_xy):
                    self.get_logger().info(
                        f'Patch id={patch.marker_id}: nicht vollstaendig in Reichweite '
                        f'({self.removal_proximity_threshold}m), warte weiter.'
                    )
                    still_valid_patches.append(patch)
                    continue
        
                # 3) Vollstaendigkeits-Check NUR gegen Live-Cluster (nicht /map):
                #    Objekt gilt als weg, wenn aktuell kein Live-Cluster mehr mit
                #    dem Blob ueberlappt.
                if self.live_cluster_overlaps_blob(patch.cells):
                    self.get_logger().info(
                        f'Patch id={patch.marker_id}: Live-Cluster ueberlappt noch '
                        f'mit dem Objekt-Blob, warte weiter.'
                    )
                    still_valid_patches.append(patch)
                    continue
        
                if self.clear_cells(patched, patch.cells):
                    applied += 1
                still_valid_patches.append(patch)
            except Exception:
                import traceback
                self.get_logger().error(
                    f'Exception beim Verarbeiten von Patch id={patch.marker_id}:\n'
                    f'{traceback.format_exc()}'
                )
                still_valid_patches.append(patch)
                continue

        self.patches = still_valid_patches

        if applied:
            self.get_logger().info(
                f'{applied} Objekt(e) vollstaendig aus der Karte entfernt.'
            )
        self.publish_debug_blobs()
        self.map_pub.publish(patched)

    def transform_to_map(
        self, patch: RemovedObjectPatch, stamp
    ) -> Optional[Tuple[float, float]]:
        return self.transform_xy_to_map(patch.frame_id, patch.x, patch.y)

    def transform_xy_to_map(
        self, frame_id: str, x: float, y: float
    ) -> Optional[Tuple[float, float]]:
        if frame_id == self.map_frame:
            return x, y

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                Time(),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Konnte Punkt nicht von "{frame_id}" nach "{self.map_frame}" transformieren: {exc}'
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        rot00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        rot01 = 2.0 * (qx * qy - qw * qz)
        rot10 = 2.0 * (qx * qy + qw * qz)
        rot11 = 1.0 - 2.0 * (qx * qx + qz * qz)

        map_x = translation.x + rot00 * x + rot01 * y
        map_y = translation.y + rot10 * x + rot11 * y
        return map_x, map_y

    def get_robot_position(self, stamp) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Konnte Roboterposition nicht in "{self.map_frame}" bestimmen: {exc}'
            )
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def resolve_object_cells(
        self, grid: OccupancyGrid, center_x: float, center_y: float, search_radius: float
    ) -> Optional[List[Tuple[float, float]]]:
        """Nimmt alle Zellen innerhalb von `search_radius` um (center_x, center_y),
        die tatsaechlich erkundet wurden (also nicht UNKNOWN sind). Das erfasst
        den kompletten Sichtfeld-Bereich des gemeldeten Objekts, auch wenn seine
        OCCUPIED-Zellen durch einzelne noch nicht erkundete Luecken unterbrochen
        sind. Gibt None zurueck, wenn im Suchradius ueberhaupt keine belegte
        Zelle gefunden wurde (Objekt vermutlich noch nicht/nicht hier)."""
        info = grid.info
        resolution = info.resolution
        if resolution <= 0.0:
            return None
    
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        data = grid.data
    
        radius = search_radius + self.blob_search_margin
        min_col = max(int(math.floor((center_x - radius - origin_x) / resolution)), 0)
        max_col = min(int(math.floor((center_x + radius - origin_x) / resolution)), width - 1)
        min_row = max(int(math.floor((center_y - radius - origin_y) / resolution)), 0)
        max_row = min(int(math.floor((center_y + radius - origin_y) / resolution)), height - 1)
    
        if min_col > max_col or min_row > max_row:
            return None
    
        radius_cells_sq = (radius / resolution) ** 2
        cells: List[Tuple[float, float]] = []
        found_occupied = False
    
        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                cell_x = origin_x + (col + 0.5) * resolution
                cell_y = origin_y + (row + 0.5) * resolution
                dx = (cell_x - center_x) / resolution
                dy = (cell_y - center_y) / resolution
                if dx * dx + dy * dy > radius_cells_sq:
                    continue
                index = row * width + col
                value = data[index]
                if value == UNKNOWN:
                    # Nie erkundet -> gehoert nicht zum "im Sichtfeld war"-Bereich.
                    continue
                if value == OCCUPIED:
                    found_occupied = True
                cells.append((cell_x, cell_y))
    
        if not found_occupied:
            # Im Suchradius war nichts belegt -> vermutlich falscher Ort/noch
            # nicht in der Karte angekommen, spaeter erneut versuchen.
            return None
    
        return cells
    
    def _gap_is_bridgeable(
        self, data, width: int, height: int, row0: int, col0: int, row1: int, col1: int
    ) -> bool:
        """Prueft die Zellen auf der direkten Linie zwischen zwei Punkten
        (exklusive der Endpunkte). Gibt False zurueck, sobald eine FREE-Zelle
        dazwischen liegt (echter freier Korridor, nicht ueberbruecken), sonst
        True (nur UNKNOWN dazwischen -> Luecke darf ueberbrueckt werden)."""
        steps = max(abs(row1 - row0), abs(col1 - col0))
        if steps <= 1:
            return True
        for step in range(1, steps):
            r = round(row0 + (row1 - row0) * step / steps)
            c = round(col0 + (col1 - col0) * step / steps)
            if not (0 <= r < height and 0 <= c < width):
                return False
            if data[r * width + c] == FREE:
                return False
        return True


    def clear_cells(self, grid: OccupancyGrid, cells: List[Tuple[float, float]]) -> bool:
        info = grid.info
        resolution = info.resolution
        if resolution <= 0.0:
            return False
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        data = list(grid.data)

        changed = False
        for x, y in cells:
            col = int(math.floor((x - origin_x) / resolution))
            row = int(math.floor((y - origin_y) / resolution))
            if 0 <= row < height and 0 <= col < width:
                index = row * width + col
                if data[index] != FREE:
                    data[index] = FREE
                    changed = True

        grid.data = data
        return changed

    def object_reappeared(self, patch: RemovedObjectPatch, map_xy: Tuple[float, float]) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        check_radius = patch.radius + self.reappear_margin
        for frame_id, x, y, stamp_sec in self.live_cluster_points:
            if now_sec - stamp_sec > self.live_cluster_max_age_sec:
                continue
            point_map_xy = self.transform_xy_to_map(frame_id, x, y)
            if point_map_xy is None:
                continue
            dx = point_map_xy[0] - map_xy[0]
            dy = point_map_xy[1] - map_xy[1]
            if math.hypot(dx, dy) <= check_radius:
                return True
        return False
    
    def object_fully_in_range(
        self, cells: List[Tuple[float, float]], robot_xy: Tuple[float, float]
    ) -> bool:
        for x, y in cells:
            if math.hypot(x - robot_xy[0], y - robot_xy[1]) > self.removal_proximity_threshold:
                return False
        return True
    
    def live_cluster_overlaps_blob(self, cells: List[Tuple[float, float]]) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        # Ueberlappungs-Toleranz: eine Zelle gilt als "getroffen", wenn ein
        # Live-Cluster-Centroid innerhalb dieses Radius um sie liegt.
        hit_radius = max(self.blob_search_margin, 0.15)
        for frame_id, x, y, stamp_sec in self.live_cluster_points:
            if now_sec - stamp_sec > self.live_cluster_max_age_sec:
                continue
            point_map_xy = self.transform_xy_to_map(frame_id, x, y)
            if point_map_xy is None:
                continue
            for cx, cy in cells:
                if math.hypot(point_map_xy[0] - cx, point_map_xy[1] - cy) <= hit_radius:
                    return True
        return False
    
    def publish_debug_blobs(self) -> None:
        marker_array = MarkerArray()
        for i, patch in enumerate(self.patches):
            if not patch.cells:
                continue
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'patch_cells'
            marker.id = patch.marker_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            for x, y in patch.cells:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.05
                marker.points.append(p)
            marker_array.markers.append(marker)
        self.debug_blob_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MapPatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()