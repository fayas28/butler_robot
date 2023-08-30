#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConfirmationPublisher(Node):
    def __init__(self):
        super().__init__('confirmation_publisher')
        self.publisher_ = self.create_publisher(String, '/confirmation', 10)
        self.get_logger().info("Confirmation Publisher node initialized")

    def publish_confirmation(self):
        confirmation_msg = String()
        confirmation_msg.data = "yes"
        self.publisher_.publish(confirmation_msg)
        self.get_logger().info("Confirmation message published: 'yes'")

def main(args=None):
    rclpy.init(args=args)
    confirmation_publisher = ConfirmationPublisher()

    while rclpy.ok():
        user_input = input("Enter 'yes' to confirm: ")
        if user_input.lower() == "yes":
            confirmation_publisher.publish_confirmation()
        else:
            print("Invalid input. Please enter 'yes' to confirm.")

if __name__ == '__main__':
    main()
