import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import numpy as np


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


class MapMergeNode(Node):
    def __init__(self):
        super().__init__('map_merge_node')

        self.declare_parameter('robot_names', ['TT_robot0', 'TT_robot1', 'TT_robot2'])
        self.declare_parameter('global_frame', 'map')

        # set up transform buffer and listener
        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._robot_names = self.get_parameter('robot_names').value
        self._global_frame = self.get_parameter('global_frame').value

        self._local_maps = {name: None for name in self._robot_names}

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('merged_map_topic', '/map')
        self.declare_parameter('publish_rate', 1.0)

        self._resolution = self.get_parameter('resolution').value

        for name in self._robot_names:
            self.create_subscription(
                OccupancyGrid, f'/{name}/map',
                lambda msg, n=name: self._map_callback(n, msg), 1
            )

        self._merged_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('merged_map_topic').value, 1)

        period = 1.0 / self.get_parameter('publish_rate').value
        self.create_timer(period, self._merge_and_publish)


        # self.create_timer(1.0, self._check_initial_poses)

    def _get_transform(self, robot_name):
        try:
            return self._tf_buffer.lookup_transform(
                self._global_frame, f'{robot_name}/map', rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug(f'TF not ready for {robot_name}: {e}')
            return None

    # def _check_initial_poses(self):
    #     """
    #     Function to check if Tf is successfully read (just for checking reading issues)
    #     """
    #     for name in self._robot_names:
    #         tf = self._get_transform(name)
    #         if tf is None:
    #             self.get_logger().info(f'{name}: transform not available yet')
    #             continue

    #         tx = tf.transform.translation.x
    #         ty = tf.transform.translation.y
    #         yaw = quaternion_to_yaw(tf.transform.rotation)

    #         self.get_logger().info(
    #             f'{name}: x={tx:.2f}, y={ty:.2f}, yaw={yaw:.2f}')

    def _map_callback(self, robot_name, msg):
        self._local_maps[robot_name] = msg

    def _local_cells_to_global_xy(self, msg, tf):
        """convert every cell of a local grid into
        global frame coordinates, applying rotation + translation."""
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = quaternion_to_yaw(tf.transform.rotation)
        cos_t, sin_t = np.cos(yaw), np.sin(yaw)

        cols = np.arange(w)
        rows = np.arange(h)
        col_grid, row_grid = np.meshgrid(cols, rows)  # shape (h, w)

        x_local = ox + (col_grid + 0.5) * res
        y_local = oy + (row_grid + 0.5) * res

        x_global = tx + cos_t * x_local - sin_t * y_local
        y_global = ty + sin_t * x_local + cos_t * y_local

        return x_global, y_global

    def _merge_and_publish(self):
        maps_with_tf = []
        for name, msg in self._local_maps.items():
            if msg is None:
                continue
            tf = self._get_transform(name)
            if tf is None:
                continue
            maps_with_tf.append((msg, tf))

        if not maps_with_tf:
            return

        # compute global bounds
        all_x_min, all_x_max = [], []
        all_y_min, all_y_max = [], []
        cached = []

        for msg, tf in maps_with_tf:
            w, h = msg.info.width, msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            x_global, y_global = self._local_cells_to_global_xy(msg, tf)

            cached.append((data, x_global, y_global))
            all_x_min.append(x_global.min())
            all_x_max.append(x_global.max())
            all_y_min.append(y_global.min())
            all_y_max.append(y_global.max())

        min_x, max_x = min(all_x_min), max(all_x_max)
        min_y, max_y = min(all_y_min), max(all_y_max)

        gw = int(np.ceil((max_x - min_x) / self._resolution)) + 1
        gh = int(np.ceil((max_y - min_y) / self._resolution)) + 1

        global_grid = np.full((gh, gw), -1, dtype=np.int8)

        # scatter cells in global grid
        for data, x_global, y_global in cached:
            valid = data != -1

            g_col = np.floor((x_global - min_x) / self._resolution).astype(np.int64)
            g_row = np.floor((y_global - min_y) / self._resolution).astype(np.int64)

            in_bounds = (
                (g_row >= 0) & (g_row < gh) &
                (g_col >= 0) & (g_col < gw)
            )
            mask = valid & in_bounds

            rows_flat = g_row[mask]
            cols_flat = g_col[mask]
            vals_flat = data[mask]

            occ_mask = vals_flat > 0
            free_mask = vals_flat == 0

            global_grid[rows_flat[occ_mask], cols_flat[occ_mask]] = vals_flat[occ_mask]

            free_rows = rows_flat[free_mask]
            free_cols = cols_flat[free_mask]
            still_unknown = global_grid[free_rows, free_cols] == -1
            global_grid[free_rows[still_unknown], free_cols[still_unknown]] = 0

        # publish global map
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._global_frame
        out.info.resolution = self._resolution
        out.info.width = gw
        out.info.height = gh
        out.info.origin.position.x = min_x
        out.info.origin.position.y = min_y
        out.info.origin.orientation.w = 1.0
        out.data = global_grid.flatten().tolist()

        self._merged_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()