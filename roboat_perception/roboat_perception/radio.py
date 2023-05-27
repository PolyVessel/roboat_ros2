import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps, core, sparkfun_predefines
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import subprocess
import os

import serial

from lib.util import TimeoutException, time_limit

import RPi.GPIO as GPIO
from lib.depacketizer import Depacketizer

class RadioResponseBad(Exception): pass

class LoRaRadio(Node):
    def __init__(self):
        super().__init__('lora_radio')

        M0 = 2
        M1 = 27
        AUX = 22

        self.serial_port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3, write_timeout=3)

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(M0, GPIO.OUT)
        GPIO.setup(M1, GPIO.OUT)
        GPIO.setup(AUX, GPIO.IN)

        timer_period = 1  # seconds
        self.radio_poll_timer = self.create_timer(timer_period, self.poll_gps)  

        #self.publisher_ = self.create_publisher(NavSatFix, '/sensor/gpsfix', 10)
        
        self.get_logger().info("LoRa Radio Initialized")

        self.get_logger().info("Initiating Self-test")
        self.ping_radio()

        self._normal_mode()
        
        self.get_logger().info("Passed Self-test")

    
    def __del__(self):
        self.serial_port.close()
        GPIO.cleanup()

    def _normal_mode(self):
        """Will block until module is free and can swap the mode"""

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.LOW)
    
    def _wake_up_mode(self):
        """Will block until module is free and can swap the mode
        
        Note: This mode will transmit a wake-up packet, so even if
        receiver is in power-saving mode, it can still hear it
        """

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.HIGH)
        GPIO.output(self.m1_pin, GPIO.LOW)

    def _power_saving_mode(self):
        """Will block until module is free and can swap the mode
        
        Can't transmit and will only listen to transmissions from
        a transmitter in wake-up mode.
        """

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.HIGH)

    def _sleep_mode(self):
        """Will block until module is free and can swap the mode"""

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.HIGH)

    def ping_radio(self):
        """Requests version number of radio
        Will return a byte-string tuple, 
            ('\x00','\x00') - Indicates error
            Otherwise, first value is Version number and 
            second value is other module features.

        Can raise TimeoutException() or RadioResponseBad()
        """
        with time_limit(3):
            self._sleep_mode()


        self.serial_port.write(b'\xC3\xC3\xC3')
        radio_resp = self.serial_port.read(4)

        if len(radio_resp) != 4:
            raise RadioResponseBad(f"Did not return correct data length! Response: {radio_resp}")

        print(f"Radio Ping response {radio_resp} with length {len(radio_resp)}")

        # Asserts that radio is a 433MHz model and 
        # received correct amount of data
        
        if (not radio_resp[0] == 0xc3):
            raise RadioResponseBad("First byte is not 0xC3! Resp: " + str(radio_resp))
        
        if (not radio_resp[1] == 0x32):
            raise RadioResponseBad("Second byte is not 0x32! Resp: " + str(radio_resp))
        
        if (not len(radio_resp) == 4):
            raise RadioResponseBad("Radio Respone is not 4 bytes long! Resp: " + str(radio_resp))
        
        return (radio_resp[2],radio_resp[3])
    
    def receive(self):
        serial_resp = self.serial_port.read(self.serial_port.in_waiting)
        return serial_resp

    def _block_until_module_free(self):
        while not GPIO.input(self.aux_pin):
            pass # Block until Aux is 1
    
    def transmit(self, data: bytes):
        if len(data) > 512:
            raise ValueError("Data too long to transmit!")
        buf = bytearray(b'\xff' * 2)
        buf.extend(data)
        self.serial_port.write(buf)

    def bytes_waiting():
        return serial.inWaiting()

        


def main(args=None):
    rclpy.init(args=args)

    gps_node = LoRaRadio()

    rclpy.spin(gps_node)

    gps_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
