#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String  #/line_follow: std_msgs/msg/String

class StringSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("line_follow_subscriber")
        self.string_subscriber_ = self.create_subscription(
            String, "/line_follow", self.string_callback, 10)

    def string_callback(self, msg: String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = StringSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
