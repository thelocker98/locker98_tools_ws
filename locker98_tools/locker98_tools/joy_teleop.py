#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import pygame
import time
from geometry_msgs.msg import Twist




class JoyTeleopNode(Node):
    def __init__(self, joystick):
        super().__init__("joy_teleop")
        self.joystick = joystick
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.publish_news)
        self.get_logger().info("Starting the robot news station")

    def publish_news(self):
        r, move = self.get_joystick_right_stick()

        msg = Twist()
        msg.linear.x = move*2
        msg.angular.z = r*2
        self.publisher_.publish(msg)

    def get_joystick_right_stick(self):
        # Pump the event queue to prevent freezing
        pygame.event.pump()
        # Get right joystick axis values
        right_x = round(self.joystick.get_axis(3), 2)/3  # Adjust index if not correct
        right_y = round(self.joystick.get_axis(4), 2)/4  # Adjust index if not correct
        if right_x < 0.02 and right_x > -0.02:
            right_x = 0.0
        if right_y < 0.02 and right_y > -0.02:
            right_y = 0.0
        return right_x, -right_y



def main (args=None):
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    # Find Controller
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        exit(0)

    # Open the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Start ROS 2 Node
    rclpy.init(args=args)
    node = JoyTeleopNode(joystick)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
