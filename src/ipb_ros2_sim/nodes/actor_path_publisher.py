#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray

class ActorPathPublisher(Node):
    def __init__(self):
        super().__init__('actor_path_publisher')

        self.declare_parameter('namespace', '')
        namespace = self.get_parameter('namespace').value
        self.topic = f"/{namespace}cmd_path"
        self.get_logger().info(f"Publishing on: {self.topic}")

        # Publisher to the actor's path topic
        self.pub = self.create_publisher(PoseArray, self.topic, 10)

        # Publish rate (Hz)
        self.timer = self.create_timer(5, self.publish_path)

        self.get_logger().info("Actor path publisher started")

        # Example path: a simple square
        self.path_points = [
            (1.0, 1.0, 0.0),
            (4.0, 1.0, 0.0),
            (4.0, 4.0, 0.0),
            (6.0, 4.0, 0.0),
            (6.0, 18.0, 0.0),
            (-2.0, 18.0, 0.0),
            (-2.0, 1.0, 0.0),
            (1.0, 1.0, 0.0),
        ]

    def publish_path(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'

        for x, y, z in self.path_points:
            pose = Pose()
            pose.position = Point(x=x, y=y, z=z)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # no rotation
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)
        self.get_logger().info(f"Published path with {len(self.path_points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = ActorPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()