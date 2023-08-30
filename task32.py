#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        self.subscription = self.create_subscription(
            String,
            '/order',
            self.order_callback,
            10
        )
        self.table_number = None
        self.get_logger().info("Order Receiver node initialized")

        self.confirmation_subscription = self.create_subscription(
            String,
            '/confirmation',
            self.confirmation_callback,
            10
        )

        self.confirmation_timer = None
        self.return_home_timer = None
        self.waiting_for_order = False

    def order_callback(self, msg):
        if not self.waiting_for_order:
            order_info = msg.data
            self.get_logger().info(f"Received order: {order_info}")
            self.table_number = order_info.split(',')[0]  
            self.deliver_to_kitchen()

    def deliver_to_kitchen(self):
        self.get_logger().info("Robot is on its way to the kitchen")
        time.sleep(3)  
        self.get_logger().info("Robot has reached the kitchen")
        self.get_logger().info("Waiting for confirmation from kitchen")
        self.waiting_for_order = True

    def confirmation_callback(self, msg):
        if self.waiting_for_order:
            if msg.data.lower() == "yes":
                self.get_logger().info("Confirmation received")
                if self.table_number:
                    self.move_to_table()
            else:
                self.get_logger().info("Confirmation not received, going back to the kitchen")
                self.return_to_kitchen()

    def move_to_table(self):
        self.get_logger().info("Robot is on its way to the table")
        time.sleep(3) 
        self.get_logger().info(f"Robot has reached Table {self.table_number}")
        self.get_logger().info("Waiting for confirmation from the table")
        self.start_confirmation_timer()

    def start_confirmation_timer(self):
        if self.confirmation_timer:
            self.confirmation_timer.cancel()

        self.confirmation_timer = self.create_timer(2, self.confirmation_timeout)

    def confirmation_timeout(self):
        self.get_logger().info("Confirmation timeout, going back to the kitchen")
        self.return_to_kitchen()

    def return_to_kitchen(self):
        self.get_logger().info("Robot is returning to the kitchen")
        time.sleep(2)  
        self.return_home()

    def return_home(self):
        self.get_logger().info("Robot is returning home")
        self.get_logger().info("Reached home")
        time.sleep(1)  
        self.waiting_for_order = False  


    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)  
            time.sleep(0.1)  

def main(args=None):
    rclpy.init(args=args)
    order_receiver = OrderReceiver()

    order_receiver.run()  

    rclpy.shutdown()

if __name__ == '__main__':
    main()
