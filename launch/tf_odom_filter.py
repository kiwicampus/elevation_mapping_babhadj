#!/usr/bin/env python3
"""
TF odom filter node.

Subscribes to /tf_bag (the bag's /tf, remapped to avoid conflict) and
republishes all transforms on /tf EXCEPT those owned by live nodes:
  - parent 'odom'        -> dropped (3D EKF publishes odom->base_link)
  - child 'inertial_link' -> dropped (average_imu publishes base_link->inertial_link
                             with real roll/pitch; bag version was identity)
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfOdomFilter(Node):
    def __init__(self):
        super().__init__("tf_odom_filter")
        self.declare_parameter("filtered_parent_frames", ["odom"])
        self.declare_parameter("filtered_child_frames", ["inertial_link"])
        self._filtered_parents = set(self.get_parameter("filtered_parent_frames").value)
        self._filtered_children = set(self.get_parameter("filtered_child_frames").value)

        self._pub = self.create_publisher(TFMessage, "/tf", 100)
        self._sub = self.create_subscription(TFMessage, "/tf_bag", self._cb, 100)

    def _cb(self, msg: TFMessage):
        kept = [
            t
            for t in msg.transforms
            if t.header.frame_id not in self._filtered_parents
            and t.child_frame_id not in self._filtered_children
        ]
        if kept:
            out = TFMessage()
            out.transforms = kept
            self._pub.publish(out)


def main():
    rclpy.init()
    node = TfOdomFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
