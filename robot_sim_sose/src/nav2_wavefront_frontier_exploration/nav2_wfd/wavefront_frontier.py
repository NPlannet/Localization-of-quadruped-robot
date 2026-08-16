#! /usr/bin/env python3
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque
import math
import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

FREE_THRESHOLD = 49
OCCUPIED_THRESHOLD = 65
MIN_FRONTIER_SIZE = 5


class OccupancyGrid2d:
    """Small adapter around nav_msgs/OccupancyGrid."""

    def __init__(
        self,
        map_msg,
        free_threshold=FREE_THRESHOLD,
        occupied_threshold=OCCUPIED_THRESHOLD,
    ):
        self.map = map_msg
        self.free_threshold = free_threshold
        self.occupied_threshold = occupied_threshold

    def getCost(self, mx, my):
        return self.map.data[my * self.map.info.width + mx]

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def isInside(self, mx, my):
        return 0 <= mx < self.getSizeX() and 0 <= my < self.getSizeY()

    def isUnknown(self, mx, my):
        return self.getCost(mx, my) < 0

    def isFree(self, mx, my):
        cost = self.getCost(mx, my)
        return 0 <= cost <= self.free_threshold

    def mapToWorld(self, mx, my):
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        return (
            origin.x + (mx + 0.5) * resolution,
            origin.y + (my + 0.5) * resolution,
        )

    def worldToMap(self, wx, wy):
        origin = self.map.info.origin.position
        if wx < origin.x or wy < origin.y:
            raise ValueError('World coordinates out of bounds')

        mx = int((wx - origin.x) / self.map.info.resolution)
        my = int((wy - origin.y) / self.map.info.resolution)
        if not self.isInside(mx, my):
            raise ValueError('World coordinates out of bounds')
        return (mx, my)


def getNeighbors(cell, costmap, include_diagonals=True):
    mx, my = cell
    neighbors = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            if not include_diagonals and dx != 0 and dy != 0:
                continue
            candidate = (mx + dx, my + dy)
            if costmap.isInside(*candidate):
                neighbors.append(candidate)
    return neighbors


def findFree(mx, my, costmap):
    """Find the nearest traversable cell to the robot's map cell."""
    queue = deque([(mx, my)])
    visited = {(mx, my)}

    while queue:
        cell = queue.popleft()
        if costmap.isFree(*cell):
            return cell
        for neighbor in getNeighbors(cell, costmap, include_diagonals=False):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return None


def isFrontierPoint(cell, costmap):
    """Return true for reachable free space bordering unexplored space."""
    if not costmap.isFree(*cell):
        return False

    mx, my = cell
    # Cartographer crops /map to the painted submaps. A free cell on that
    # boundary is therefore also adjacent to space that is not represented yet.
    if mx in (0, costmap.getSizeX() - 1):
        return True
    if my in (0, costmap.getSizeY() - 1):
        return True

    return any(
        costmap.isUnknown(*neighbor)
        for neighbor in getNeighbors(cell, costmap, include_diagonals=False)
    )


def clusterFrontiers(frontier_cells, costmap, minimum_size=MIN_FRONTIER_SIZE):
    remaining = set(frontier_cells)
    clusters = []

    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        cluster = [seed]

        while queue:
            cell = queue.popleft()
            for neighbor in getNeighbors(cell, costmap):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)

        if len(cluster) >= minimum_size:
            clusters.append(cluster)
    return clusters


def frontierGoal(cluster, costmap, robot_cell):
    """Choose an actual free frontier cell, never an unknown centroid."""
    mean_x = sum(cell[0] for cell in cluster) / len(cluster)
    mean_y = sum(cell[1] for cell in cluster) / len(cluster)
    goal_cell = max(
        cluster,
        key=lambda cell: (
            math.hypot(cell[0] - robot_cell[0], cell[1] - robot_cell[1]),
            -math.hypot(cell[0] - mean_x, cell[1] - mean_y),
        ),
    )
    return costmap.mapToWorld(*goal_cell)


def getFrontier(pose, costmap, logger=None, minimum_size=MIN_FRONTIER_SIZE):
    """Find reachable free-space frontier goals in world coordinates."""
    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    start = findFree(mx, my, costmap)
    if start is None:
        return []

    queue = deque([start])
    visited = {start}
    frontier_cells = set()

    while queue:
        cell = queue.popleft()
        if isFrontierPoint(cell, costmap):
            frontier_cells.add(cell)

        for neighbor in getNeighbors(cell, costmap, include_diagonals=False):
            if neighbor not in visited and costmap.isFree(*neighbor):
                visited.add(neighbor)
                queue.append(neighbor)

    clusters = clusterFrontiers(frontier_cells, costmap, minimum_size)
    return [frontierGoal(cluster, costmap, start) for cluster in clusters]


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__(node_name='nav2_frontier_explorer', namespace='')
        self.free_threshold = self.declare_parameter(
            'free_threshold', FREE_THRESHOLD
        ).value
        self.occupied_threshold = self.declare_parameter(
            'occupied_threshold', OCCUPIED_THRESHOLD
        ).value
        self.minimum_frontier_size = self.declare_parameter(
            'minimum_frontier_size', MIN_FRONTIER_SIZE
        ).value
        self.minimum_goal_distance = self.declare_parameter(
            'minimum_goal_distance', 0.50
        ).value
        self.empty_frontier_retries = self.declare_parameter(
            'empty_frontier_retries', 10
        ).value

        self.currentPose = None
        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        self.navigator_state_client = self.create_client(
            GetState, '/bt_navigator/get_state'
        )
        self.goal_handle = None
        self.failed_goals = []
        self.empty_count = 0
        self.map_message_count = 0

        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.costmapSub = self.create_subscription(
            OccupancyGrid, '/map', self.occupancyGridCallback, map_qos
        )
        self.costmap = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('Frontier explorer started')

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(
            msg,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
        )
        self.map_message_count += 1
        if self.map_message_count == 1 or self.map_message_count % 10 == 0:
            unknown = sum(value < 0 for value in msg.data)
            free = sum(
                0 <= value <= self.free_threshold for value in msg.data
            )
            occupied = sum(
                value >= self.occupied_threshold for value in msg.data
            )
            uncertain = len(msg.data) - unknown - free - occupied
            self.info_msg(
                f'Map {msg.info.width}x{msg.info.height}: '
                f'free={free}, unknown={unknown}, uncertain={uncertain}, '
                f'occupied={occupied}'
            )

    def waitForNavigation(self):
        while rclpy.ok():
            if not self.navigator_state_client.wait_for_service(timeout_sec=1.0):
                self.info_msg('Waiting for Nav2 BT navigator lifecycle service...')
                continue

            future = self.navigator_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if (
                response is not None
                and response.current_state.id == State.PRIMARY_STATE_ACTIVE
                and self.action_client.wait_for_server(timeout_sec=1.0)
            ):
                self.info_msg('Nav2 navigate-to-pose server is active')
                return True
            self.info_msg('Waiting for Nav2 BT navigator to become active...')
        return False

    def getCurrentMapPose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.warn_msg(f'Waiting for map -> base_link transform: {exc}')
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def selectFrontier(self, frontiers):
        candidates = []
        for frontier in frontiers:
            distance = math.hypot(
                frontier[0] - self.currentPose.position.x,
                frontier[1] - self.currentPose.position.y,
            )
            if distance < self.minimum_goal_distance:
                continue
            if any(
                math.hypot(frontier[0] - failed[0], frontier[1] - failed[1])
                < self.minimum_goal_distance
                for failed in self.failed_goals
            ):
                continue
            candidates.append((distance, frontier))

        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def moveToFrontiers(self):
        self.currentPose = self.getCurrentMapPose()
        if self.currentPose is None:
            return None

        try:
            frontiers = getFrontier(
                self.currentPose,
                self.costmap,
                self.get_logger(),
                self.minimum_frontier_size,
            )
        except (IndexError, ValueError) as exc:
            self.warn_msg(f'Cannot evaluate the current map yet: {exc}')
            return None

        location = self.selectFrontier(frontiers)
        if location is None:
            self.empty_count += 1
            if self.empty_count <= self.empty_frontier_retries:
                self.info_msg(
                    'No usable frontier yet; waiting for an updated map '
                    f'({self.empty_count}/{self.empty_frontier_retries})'
                )
                return None
            self.info_msg('No More Frontiers')
            return False

        self.empty_count = 0
        distance = math.hypot(
            location[0] - self.currentPose.position.x,
            location[1] - self.currentPose.position.y,
        )
        self.info_msg(
            f'Navigating to free frontier ({location[0]:.2f}, '
            f'{location[1]:.2f}), distance={distance:.2f} m'
        )

        goal = NavigateToPose.Goal()
        goal.pose = self.makeGoalPose(location)
        self.info_msg('Sending goal request...')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self.goal_handle = send_future.result()

        if self.goal_handle is None or not self.goal_handle.accepted:
            self.error_msg('Goal rejected; trying another frontier')
            self.failed_goals.append(location)
            return None

        self.info_msg('Goal accepted')
        result_future = self.goal_handle.get_result_async()
        self.info_msg("Waiting for '/navigate_to_pose' action to complete")
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()
        if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
            status = response.status if response is not None else 'no result'
            self.error_msg(
                f'Frontier goal failed with status {status}; trying another'
            )
            self.failed_goals.append(location)
            return None

        self.info_msg('Frontier goal reached; evaluating the updated map')
        return True

    def makeGoalPose(self, location):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = location[0]
        goal.pose.position.y = location[1]
        yaw = math.atan2(
            location[1] - self.currentPose.position.y,
            location[0] - self.currentPose.position.x,
        )
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def info_msg(self, msg):
        self.get_logger().info(msg)

    def warn_msg(self, msg):
        self.get_logger().warn(msg)

    def error_msg(self, msg):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    explorer = FrontierExplorer()

    if not explorer.waitForNavigation():
        explorer.destroy_node()
        rclpy.shutdown()
        return

    while rclpy.ok() and explorer.costmap is None:
        explorer.info_msg('Getting initial map')
        rclpy.spin_once(explorer, timeout_sec=1.0)

    while rclpy.ok():
        result = explorer.moveToFrontiers()
        if result is False:
            break
        rclpy.spin_once(explorer, timeout_sec=2.0)

    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
