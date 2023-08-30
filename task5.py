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
        self.table_orders = []
        self.get_logger().info("Order Receiver node initialized")

    def order_callback(self, msg):
        order_info = msg.data
        self.get_logger().info(f"Received order: {order_info}")
        self.table_orders.append(order_info.split(',')[0])  
        
        if len(self.table_orders) == 3:
            self.process_orders()

    def process_orders(self):
        self.get_logger().info("Processing orders...")
        self.move_to_table()
        self.return_home()

    def deliver_to_kitchen(self):
        self.get_logger().info("Robot is on its way to the kitchen")
        time.sleep(3)  
        self.get_logger().info("Robot has reached the kitchen")


    #iterating through each elements in the list
    def move_to_table(self):
        for table_number in self.table_orders:
            self.get_logger().info(f"Robot is on its way to Table {table_number}")
            time.sleep(2)  
            self.get_logger().info(f"Robot has reached Table {table_number}")

    def return_home(self):
        self.get_logger().info("Robot is on its way back home")
        time.sleep(3)  
        self.get_logger().info("Robot has reached home position")
        time.sleep(3)  
        self.table_orders = []  

    def run(self):
        self.get_logger().info("Waiting for next order")
        while rclpy.ok():
            rclpy.spin_once(self)  
            time.sleep(0.1)  

def main(args=None):
    rclpy.init(args=args)
    order_receiver = OrderReceiver()

    order_receiver.run() 

    order_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
