#!/usr/bin/env python3

'''
Publisher Node to simulate a robot following a black line on a white surface using IR sensor
- GPS Location Publisher Node
    GPS coordinates (latitude, longitude)
        Latitude to increase incrementally
- Motor (Left/Right and Speed) Publisher Node
    Left/Right motor speed (0-100)
    If IR sensor detects white surface, left or right motor speed is set to 100 to turn the robot
    If IR sensor detects black line, both motor speeds are set to 50 to move forward
- IR Sensor Publisher Node
    0 = white surface
    1 = black line
'''

import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__('line_follow')
        
        self.follow = self.create_publisher(String, 'line_follow', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.ir = 1 # 0 = white surface, 1 = black line
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        
        self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        
        self.ir = random.randint(0, 1)
        
        if self.ir == 1: # black line
            self.left_motor_speed = 50
            self.right_motor_speed = 50
            self.x += 1 # x change for demonstration purposes only
        else: # white surface
            self.left_motor_speed = 100
            self.right_motor_speed = 0
            self.y += 1 # y change for demonstration purposes only

        msg = String()
        msg.data = f"x: {self.x}, y: {self.y}, IR: {self.ir}, Left Motor: {self.left_motor_speed}, Right Motor: {self.right_motor_speed}"
        self.follow.publish(msg)
        # self.get_logger().info(
        #     f"x={self.x}, y={self.y}, IR={self.ir}, "
        #     f"Motors=({self.left_motor_speed}, {self.right_motor_speed})")

        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    