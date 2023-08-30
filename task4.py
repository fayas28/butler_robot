#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from butler_robot.srv import CancelOrder 
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

        #program reacts to service calls in this program. Once the service is called, the robot cancels its operation and return home
        self.cancel_service = self.create_service(CancelOrder, '/cancel_order', self.cancel_order_callback)
        self.confirmation_timer = None
        self.movement_timer = None
        self.waiting_for_order = False
        self.cancellation_requested = False
        self.in_transit_to_kitchen = False
        self.in_transit_to_table = False

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

    def cancel_order_callback(self, request, response):
        if not self.waiting_for_order:
            response.success = False
            return response
        
        self.get_logger().info("Cancellation requested")
        self.cancellation_requested = True
        
        #when cancelled while going to the kitchen, it returns home while when cancelled while going to the table for serving, it moves to the kitchen and then to home position.
        if self.in_transit_to_kitchen:
            self.get_logger().info("Cancelling while going to the kitchen")
            self.return_home()
        elif self.in_transit_to_table:
            self.get_logger().info("Cancelling while going to the table")
            self.move_to_kitchen()

        response.success = True
        return response

    def move_to_table(self):
        self.get_logger().info("Robot is on its way to the table")
        time.sleep(3)
        self.get_logger().info(f"Robot has reached Table {self.table_number}")
        self.get_logger().info("Waiting for confirmation from the table")
        self.start_confirmation_timer()

    def move_to_kitchen(self):
        self.get_logger().info("Returning to the kitchen")
        time.sleep(2)
        self.return_to_kitchen()

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
        self.cancellation_requested = False
        self.waiting_for_order = False
        self.in_transit_to_kitchen = False
        self.in_transit_to_table = False
        self.get_logger().info("Waiting for next order")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            
            if self.cancellation_requested:
                self.cancellation_requested = False
                self.return_to_kitchen()

            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    order_receiver = OrderReceiver()
    order_receiver.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
