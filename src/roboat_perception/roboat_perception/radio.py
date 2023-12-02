import serial
import RPi.GPIO as GPIO
from time import sleep
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
        
        timer_period = 30
        self.timer = self.create_timer(timer_period, self.publish) # Temp Timer
        
        self.self_test()
        self.get_logger().info("Radio Initialized")
    
    def publish(self):
        msg = String()
        msg.data = '000001' # Replace with Ported Data && Convert to Serialized Format
        self.publisher_.publish(msg)
        self.get_logger().info("Message Sent")
    
    def listener_callback(self,rcvdMSG):
        self.get_logger().info("Message Recieved: " + rcvdMSG.data) # Add Unserialization Function
    
    def self_test(self):
        M0 = 0
        M1 = 2

        AUX = 3

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(M0, GPIO.OUT)
        GPIO.setup(M1, GPIO.OUT)
        GPIO.setup(AUX, GPIO.IN)

        radio_ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=2)

        GPIO.output(M0, GPIO.HIGH)
        GPIO.output(M1, GPIO.HIGH)


        sleep(1)

        while GPIO.input(AUX) == GPIO.LOW:
                pass

        radio_ser.write(b'\xC3\xC3\xC3')

        rev = radio_ser.read(size=6)

        success = False
        for s in rev:
                print(f"0x{s:02x}")
                if s == 195:
                    success = False
        if success == False:
            radio.destroy_node()
            rclpy.shutdown()
            print("you shouldn't see this")
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)

    radio = Radio()
    
    rclpy.spin(radio)

    radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
