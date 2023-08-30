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
        self.confirmation_subscriptions = {}
        self.table_orders = {}
        self.cancellation_requested = False
        self.get_logger().info("Order Receiver node initialized")

    def order_callback(self, msg):
        order_info = msg.data
        self.get_logger().info(f"Received order: {order_info}")
        table_number = order_info.split(',')[0]
        self.table_orders[table_number] = False
        self.subscribe_to_confirmation_topic(table_number)
        
        if len(self.table_orders) == 3:
            self.process_orders()

    def subscribe_to_confirmation_topic(self, table_number):
        self.confirmation_subscriptions[table_number] = self.create_subscription(
            String,
            f'/confirmation_{table_number}',
            self.confirmation_callback,
            10
        )

    def process_orders(self):
        self.get_logger().info("Processing orders...")
        self.move_to_table()
        self.return_home()

    def move_to_table(self):
        for table_number, confirmation_received in self.table_orders.items():
            if self.cancellation_requested and table_number == 'table2':
                self.get_logger().info(f"Skipping table {table_number} due to cancellation")
                continue
            
            self.get_logger().info(f"Robot is on its way to Table {table_number}")
            self.wait_for_confirmation(table_number)
            time.sleep(2)  
            self.get_logger().info(f"Robot has reached Table {table_number}")

    def wait_for_confirmation(self, table_number):
        start_time = time.time()
        while not self.table_orders[table_number] and (time.time() - start_time) < 5:
            rclpy.spin_once(self)
            time.sleep(0.1)  

    def confirmation_callback(self, msg):
        table_number = msg.topic.split('_')[-1]
        self.get_logger().info(f"Received confirmation from Table {table_number}: {msg.data}")
        self.table_orders[table_number] = True

    def cancel_order(self, table_number):
        self.get_logger().info(f"Cancelling order for table {table_number}")
        if table_number in self.table_orders:
            del self.table_orders[table_number]

    def return_home(self):
        self.get_logger().info("Robot is on its way back home")
        time.sleep(3)  
        self.get_logger().info("Robot has reached home position")
        time.sleep(3)  
        self.table_orders = {}  
        self.cancellation_requested = False

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
