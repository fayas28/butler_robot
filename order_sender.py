#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderSender(Node):
    def __init__(self):
        super().__init__('order_sender')
        self.publisher_ = self.create_publisher(String, '/order', 10)
        self.get_logger().info("Order Sender node initialized")

    #The orders were being sent where the table number and the order is the input which are both strings.
    def send_order(self, table_number, order):
        order_msg = String()
        order_msg.data = f"Table: {table_number}, Order: {order}"
        self.publisher_.publish(order_msg)
        self.get_logger().info(f"Order sent for table {table_number}: {order}")

def main(args=None):
    rclpy.init(args=args)
    order_sender = OrderSender()  #object initialsed

    while rclpy.ok():
        try:
            table_number = input("Enter table number (1, 2, or 3): ")
            order = input("Enter order: ")
            if table_number in ['1', '2', '3']:   #condition to check from a list of given tables
                order_sender.send_order(table_number, order)
            else:
                print("Invalid table number. Please enter 1, 2, or 3.")
        except ValueError:
            print("Invalid input. Please enter valid values.")

if __name__ == '__main__':
    main()
