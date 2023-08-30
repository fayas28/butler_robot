#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        self.subscription = self.create_subscription(   #Creating a subscriber for taking orders from another that I created
            String,
            '/order',
            self.order_callback,
            10
        )
        self.table_number = None
        self.get_logger().info("Order Receiver node initialized")


    #checks for the latest orders from the /order topic and upadtes them and takes out the tabke number
    def order_callback(self, msg):
        order_info = msg.data
        self.get_logger().info(f"Received order: {order_info}")
        self.table_number = order_info.split(',')[0]  #Taking out the table number
        self.deliver_to_kitchen()

    #the order received is fetched by the robot and goes to the kitchen to take the orders
    def deliver_to_kitchen(self):
        self.get_logger().info("Robot is on its way to the kitchen")
        time.sleep(3)  
        self.get_logger().info("Robot has reached the kitchen")
        self.move_to_table()

    #Once the orders are taken, it moves towards the table which has taken the order
    def move_to_table(self):
        self.get_logger().info("Robot is on its way to the table")
        time.sleep(2)  
        self.get_logger().info(f"Robot has reached Table {self.table_number}")
        self.return_home()

    #finally return home
    def return_home(self):
        self.get_logger().info("Robot is on its way back home")
        time.sleep(3)  
        self.get_logger().info("Robot has reached home position")
        time.sleep(3)  

    def run(self):
        self.get_logger().info("Waiting for next order")
        while rclpy.ok():
            rclpy.spin_once(self)  # Process any pending messages
            time.sleep(0.1)  

def main(args=None):
    rclpy.init(args=args)
    order_receiver = OrderReceiver()

    order_receiver.run()  

    order_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
