import rclpy
import tf2_ros
from rclpy.node import Node

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

        self.create_timer(1.0, self._check_initial_poses)

    def _get_transform(self, robot_name):
        try:
            return self._tf_buffer.lookup_transform(
                self._global_frame, f'{robot_name}/map', rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug(f'TF not ready for {robot_name}: {e}')
            return None

    def _check_initial_poses(self):
        """
        Function to check if Tf is successfully read (just for checking reading issues)
        """
        for name in self._robot_names:
            tf = self._get_transform(name)
            if tf is None:
                self.get_logger().info(f'{name}: transform not available yet')
                continue

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = quaternion_to_yaw(tf.transform.rotation)

            self.get_logger().info(
                f'{name}: x={tx:.2f}, y={ty:.2f}, yaw={yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()