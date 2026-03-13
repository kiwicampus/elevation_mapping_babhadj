#!/usr/bin/env python3
"""
TF odom filter node.

Subscribes to /tf_bag (the bag's /tf, remapped to avoid conflict) and
republishes all transforms on /tf EXCEPT those owned by live nodes:
  - child 'inertial_link' -> dropped (inertial_link_broadcaster publishes base_link->inertial_link
                             with real roll/pitch or identity; bag version was identity)
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class TfOdomFilter(Node):
    def __init__(self):
        super().__init__("tf_odom_filter")
        self.declare_parameter("filtered_parent_frames", [""])
        self.declare_parameter("filtered_child_frames", ["inertial_link"])
        self._filtered_parents = set(self.get_parameter("filtered_parent_frames").value)
        self._filtered_children = set(self.get_parameter("filtered_child_frames").value)

        qos_static = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self._pub_tf = self.create_publisher(TFMessage, "/tf", 100)
        self._pub_tf_static = self.create_publisher(TFMessage, "/tf_static", qos_static)
        
        self._sub_tf = self.create_subscription(TFMessage, "/tf_bag", self._cb_tf, 100)
        self._sub_tf_static = self.create_subscription(TFMessage, "/tf_static_bag", self._cb_tf_static, qos_static)

    def _filter_msg(self, msg: TFMessage) -> TFMessage:
        kept = [
            t
            for t in msg.transforms
            if t.header.frame_id not in self._filtered_parents
            and t.child_frame_id not in self._filtered_children
        ]
        if kept:
            out = TFMessage()
            out.transforms = kept
            return out
        return None

    def _cb_tf(self, msg: TFMessage):
        filtered = self._filter_msg(msg)
        if filtered:
            self._pub_tf.publish(filtered)

    def _cb_tf_static(self, msg: TFMessage):
        filtered = self._filter_msg(msg)
        if filtered:
            self._pub_tf_static.publish(filtered)




def main():
    rclpy.init()
    node = TfOdomFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
