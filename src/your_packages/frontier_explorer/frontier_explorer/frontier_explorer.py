import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker

import numpy as np
import math
from collections import deque
import subprocess
import time
import os


# imports from utils.py
from frontier_explorer.utils import (
    find_frontiers,
    cluster_frontiers,
    frontier_centroid,
    compute_information_gain,
    compute_motion_cost,
    compute_utility,
    map_to_world,
    world_to_map,
)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        # Add test mode parameter
        self.declare_parameter('test_mode', False)  # Set to True to skip Nav2
        self._test_mode = self.get_parameter('test_mode').value

        # parameter to visualize the frontier goals
        self._marker_pub = self.create_publisher(Marker, 'frontier_goal', 10)
        
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius',    0.3)
        self.declare_parameter('poll_period',       1.5)
        self.declare_parameter('map_topic',         '/map')
        self.declare_parameter('action_name',       'navigate_to_pose')
        self.declare_parameter('goal_frame',        'map')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('min_goal_distance', 0.50)
        self.declare_parameter('map_save_path',     '')

        self.declare_parameter('sensor_range', 2.0)  # meters
        self._sensor_range = self.get_parameter('sensor_range').value

        self._min_size      = self.get_parameter('min_frontier_size').value
        self._revisit_r     = self.get_parameter('revisit_radius').value
        self._goal_frame    = self.get_parameter('goal_frame').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._min_goal_dist = self.get_parameter('min_goal_distance').value
        self._map_save_path = self.get_parameter('map_save_path').value.strip()
        map_topic           = self.get_parameter('map_topic').value
        action_name         = self.get_parameter('action_name').value
        poll_period         = self.get_parameter('poll_period').value

        # ------------------------------------------------------------------
        # TF buffer for robot pose lookup
        # ------------------------------------------------------------------
        self._tf_buffer   = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._map: OccupancyGrid | None = None
        self._navigating = False
        self._visited: list[tuple[float, float]] = []
        self._current_goal: tuple[float, float] | None = None
        self._goal_sent_time: float = 0.0
        self._iteration = 0
        self._map_saved = False

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, 1)

        # Only wait for Nav2 server if not in test mode
        if not self._test_mode:
            self.get_logger().info('Waiting for Nav2 action server...')
            self._nav_client.wait_for_server()
            self.get_logger().info('Ready. Waiting for map...')
        else:
            self.get_logger().info('TEST MODE: Skipping Nav2 server. Will log goals instead.')

        self.create_timer(poll_period, self._explore)

    # ------------------------------------------------------------------
    # Publish goal marker on Rviz for verification
    # ------------------------------------------------------------------
    def _publish_goal_marker(self, x: float, y: float):
        marker = Marker()
        marker.header.frame_id = self._goal_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self._marker_pub.publish(marker)


    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    # ------------------------------------------------------------------
    # Main exploration loop
    # ------------------------------------------------------------------
    def _explore(self):
        if self._map is None or self._navigating:
            return
        
        self.get_logger().info(  # 👈 add here
            f'Map origin: {self._map.info.origin.position.x:.2f}, '
            f'{self._map.info.origin.position.y:.2f}, '
            f'resolution: {self._map.info.resolution}'
        )

        frontiers = self._find_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers — exploration complete.')
            self._save_map_once()
            return

        goal = self._best_frontier(frontiers)
        if goal is None:
            if self._robot_position() is None:
                self.get_logger().debug('TF not ready yet, retrying...')
                return
            self.get_logger().info('All frontiers already visited. Exploration complete!')
            self._finish_exploration()
            return

        self._iteration += 1
        if self._map_save_path and self._iteration % 10 == 0:
            self.get_logger().info(f'Progressively auto-saving map to {self._map_save_path} ...')
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        self.get_logger().info(
            f'Navigating to frontier ({goal[0]:.2f}, {goal[1]:.2f})')
        self._current_goal = goal
        self._publish_goal_marker(*goal)
        self._send_goal(*goal)

    # ------------------------------------------------------------------
    # Map saving and shutdown
    # ------------------------------------------------------------------
    def _finish_exploration(self):
        if self._map_save_path:
            self.get_logger().info(f'Final map save to {self._map_save_path} ...')
            try:
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path],
                    check=True
                )
                self.get_logger().info('Map saved successfully.')
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'Failed to save map: {e}')
        else:
            self.get_logger().info('No map_save_path provided. Skipping auto-save.')
            
        self.get_logger().info('Shutting down explorer.')
        raise SystemExit(0)
    
    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------
    def _find_frontiers(self) -> list[tuple[float, float]]:
        msg = self._map
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.get_logger().info(
            f'Map corners: TL={data[0,0]} TR={data[0,-1]} BL={data[-1,0]} BR={data[-1,-1]}'
        )
        
        # Use your utility function to find frontier cells
        frontier_cells = find_frontiers(data)
        
        if len(frontier_cells) == 0:
            return []
        
        # Cluster them
        clusters = cluster_frontiers(frontier_cells, cluster_size=self._min_size)
        if not clusters:
            return []
        
        # Convert cluster centroids from map coordinates to world coordinates
        centroids = []
        for cluster in clusters:
            cell_centroid = frontier_centroid(cluster)
            cy, cx = int(cell_centroid[0]), int(cell_centroid[1])  # extract here
            world_pos = map_to_world(cell_centroid, (ox, oy), res)
            self.get_logger().info(
                f'cell ({cy},{cx}) val={data[cy, cx]} '
                f'neighbours: up={data[cy-1, cx]} down={data[cy+1, cx]} '
                f'left={data[cy, cx-1]} right={data[cy, cx+1]}'
            )
            self.get_logger().info(f'world_pos: {world_pos}')
            centroids.append(world_pos)
        
        return centroids
    

    def _best_frontier(self, frontiers):
        pos = self._robot_position()
        if pos is None:
            return None
        
        rx, ry = pos
        best, best_score = None, float('-inf')
        
        # Get sensor range in cells (e.g., 2m at 0.05m/cell = 40 cells)
        sensor_range_cells = int(self._sensor_range / self._map.info.resolution)
        
        for fx, fy in frontiers:
            if self._already_visited(fx, fy):
                continue
            
            motion_cost = compute_motion_cost((rx, ry), (fx, fy))
            if motion_cost < self._min_goal_dist:
                continue
            
            # Convert world coords back to map coords to compute info gain
            frontier_cell = world_to_map((fx, fy), 
                                     (self._map.info.origin.position.x, 
                                      self._map.info.origin.position.y),
                                     self._map.info.resolution)
            
            # Compute info gain using your entropy-based function
            data = np.array(self._map.data, dtype=np.int8).reshape(
                (self._map.info.height, self._map.info.width))
            info_gains = compute_information_gain(
                data, 
                np.array([frontier_cell]), 
                sensor_range_cells
            )
            info_gain = info_gains[tuple(frontier_cell)]
            
            utility = compute_utility(info_gain, motion_cost, lambda_=0.4)
            if utility > best_score:
                best_score, best = utility, (fx, fy)
        
        return best
    
    def _already_visited(self, fx, fy):
        return any(
            math.hypot(fx - vx, fy - vy) < self._revisit_r
            for vx, vy in self._visited)

    def _robot_position(self):
        """Return (x, y) of base_link in the map frame via TF lookup."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._goal_frame, self._base_frame, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception:
            # TF not yet available.
            self.get_logger().debug(
                f'Waiting for TF {self._goal_frame} -> {self._base_frame}')
            return None

    # ------------------------------------------------------------------
    # Nav2 goal
    # ------------------------------------------------------------------
    def _send_goal(self, x: float, y: float):
        self._navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        if self._test_mode:
            self.get_logger().info(f'[TEST MODE] Goal message created: x={x:.2f}, y={y:.2f}, frame={self._goal_frame}')
            self._publish_goal_marker(x, y)
            self._visited.append((x, y))
            time.sleep(3.0)  #  pause to visualize
            self._navigating = False
        else:
            self._goal_sent_time = time.monotonic()
            future = self._nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._goal_response_cb)

    def _save_map_once(self):
        if self._map_saved or not self._map_save_path:
            return

        save_prefix = os.path.expanduser(self._map_save_path)
        save_dir = os.path.dirname(save_prefix)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        self.get_logger().info(f'Auto-saving map to: {save_prefix}')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save_prefix],
                check=True,
            )
            self._map_saved = True
            self.get_logger().info('Map saved successfully.')
        except Exception as exc:
            self.get_logger().error(f'Failed to save map: {exc}')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected. Trying next frontier.')
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        elapsed = time.monotonic() - self._goal_sent_time
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if elapsed < 0.5:
                self.get_logger().warn(
                    f'Spurious success in {elapsed:.2f}s — frontier not counted.')
            else:
                self.get_logger().info('Frontier reached. Searching for next...')
                if self._current_goal is not None:
                    self._visited.append(self._current_goal)
        else:
            self.get_logger().warn(f'Navigation failed (status={status}).')
        self._navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()