from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class Radio(Node):
    def __init__(self):
        super().__init__('radio')

        self.publisher_ = self.create_publisher(String, 'toTopic', 10)
        self.subscription = self.create_subscription(String, 'fromTopic', self.listener_callback, 10)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish) # Temp Timer
        
        self.get_logger().info("Radio Initialized")
    
    def publish(self):
        msg = String()
        msg.data = '000001' # Replace with Ported Data && Convert to Serialized Format
        self.publisher_.publish(msg)
        self.get_logger().info("Message Sent")
    
    def listener_callback(self,rcvdMSG):
        self.get_logger().info("Message Recieved: " + rcvdMSG.data) # Add Unserialization Function

def main(args=None):
    rclpy.init(args=args)

    radio = Radio()
    
    rclpy.spin(radio)

    radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
