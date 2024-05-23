import pyfirmata

from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps, core, sparkfun_predefines
from std_msgs.msg import Bool
import subprocess
import os

class Motor(Node):
    def __init__(self) -> None:
        super().__init__('motor')
        self.board = pyfirmata.ArduinoMega('/dev/USB0')
        
        # Use Servo
        self.left_motor = self.board.get_pin('a:2:p')
        self.right_motor = self.board.get_pin('a:3:p')
        
        self.enabled = True
        
        self.enabled_status = self.create_subscription(Bool, '/signals/enabled', self.enabled_handler,  10)

    def enabled_handler(self, msg: Bool):
        self.enabled = msg.data

def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    motor_node = Motor()
    
    rclpy.spin()

    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
