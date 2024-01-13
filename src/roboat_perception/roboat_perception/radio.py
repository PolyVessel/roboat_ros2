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

        self.publisher_ = self.create_publisher(String, 'recv', 10)
        self.subscription = self.create_subscription(String, 'send', self.listener_callback, 10)
        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.publish) # Temp Timer

        self.M0 = 17
        self.M1 = 27
        self.AUX = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)
        GPIO.setup(self.AUX, GPIO.IN)
        self.radio_ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=2)

        self.self_test()
        self.get_logger().info("Radio Initialized")
        self.set_power_saving_mode()
    
    def publish(self):
        self.get_logger().info("Started Reading")
        if GPIO.input(self.AUX) == GPIO.LOW:
            self.block_until_radio_ready()
            msg = self.radio_ser.read(size=10)
            if len(msg>2):
                sendMSG.data = msg.decode('utf-8')
                self.publisher_.publish(msg)
                self.get_logger().info("Message Recieved")
        else:
            self.get_logger().info("No Message Recieved")

    
    def listener_callback(self,sendMSG):
        self.set_wake_up_mode()
        sleep(1)
        self.block_until_radio_ready()
        self.get_logger().info("Sending Message: " + sendMSG.data) 
        encoded = sendMSG.data.encode('utf-8')
        self.radio_ser.write(encoded)
        self.get_logger().info("Message Sent")
        self.set_power_saving_mode()
    
    def self_test(self):
        self.set_sleep_mode()

        sleep(1)

        self.block_until_radio_ready()

        self.radio_ser.write(b'\xC3\xC3\xC3')

        rev = self.radio_ser.read(size=4)

        if len(rev) != 4:
            self.get_logger().info(f"Not Expected Byte Count, Got {rev}: Terminating")
            raise SystemExit
        elif rev[0] != 0xC3:
            self.get_logger().info(f"Not Expected First Byte, Got {rev}: Terminating")
            raise SystemExit

    def set_sleep_mode(self):
        GPIO.output(self.M0, GPIO.HIGH)
        GPIO.output(self.M1, GPIO.HIGH)
    
    def set_power_saving_mode(self):
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.HIGH)
    
    def set_wake_up_mode(self):
        GPIO.output(self.M0, GPIO.HIGH)
        GPIO.output(self.M1, GPIO.LOW)
    
    def set_normal_mode(self):
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)
    
    def block_until_radio_ready(self):
        while GPIO.input(self.AUX) == GPIO.LOW:
                pass

    def __del__(self):
        GPIO.cleanup()



def main(args=None):
    rclpy.init(args=args)

    radio = Radio()
    
    try:
        rclpy.spin(radio)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
