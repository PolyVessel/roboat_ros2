import serial
import RPi.GPIO as GPIO
from time import sleep
from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from .data.dtp import create_packet, decode_packet
from .data.sliplib import Driver
from bitstring import Bits, BitArray

class Radio(Node):
    def __init__(self):
        super().__init__('radio')
        self.slip_driver=Driver() #sliplib

        self.publisher_ = self.create_publisher(String, 'recv', 10)
        self.subscription = self.create_subscription(String, 'send', self.write_to_radio, 10)

        self.M0 = 17 #eletrical pin numbers
        self.M1 = 27
        self.AUX = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)
        GPIO.setup(self.AUX, GPIO.IN)
        self.radio_ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=0)

        self.self_test()
        self.get_logger().info("Radio Initialized")
        self.set_normal_mode()

        timer_period_sec = 1
        self.timer = self.create_timer(timer_period_sec, self.read_from_radio) # Temperary Timer
    
    def read_from_radio(self): # Needs to be tested
        buffer = self.radio_ser.read(size=100)
            #insert checks to see if radio sent data to buffer
        message_list = self.slip_driver.receive(buffer)
        for encoded_message in message_list:
            decoded = decode_packet(BitArray(encoded_message))
            if decoded is None:
                self.get_logger().error(f"Decode failed {encoded_message}")
                continue
            msg = decoded.data.bytes.decode('utf-8')
            
            self.get_logger().info("Message Recieved: " + msg)
            self.publisher_.publish(msg)


    
    def write_to_radio(self,sendMSG):
        self.block_until_radio_ready()
        self.get_logger().info("Sending Message: " + sendMSG.data) 
        
        msg_bits = Bits(sendMSG.data.encode('utf-8'))
        packet = self.slip_driver.send(create_packet(msg_bits).bytes)
         
        self.radio_ser.write(packet)
        self.get_logger().info("Message Sent")

    def self_test(self):
        self.set_sleep_mode()

        sleep(1)

        self.block_until_radio_ready()

        self.radio_ser.write(b'\xC3\xC3\xC3')
        
        sleep(0.5)

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
