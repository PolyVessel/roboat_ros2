import pyfirmata2 as firmata

from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps, core, sparkfun_predefines
from std_msgs.msg import Bool, Float32
import subprocess
import os

class Motor(Node):
    def __init__(self) -> None:
        super().__init__('motor')
        self.board = firmata.ArduinoMega('/dev/USB0')
        
        # Use Servo
        self.left_motor = self.board.get_pin('d:2:s')
        self.board.servo_config(2, 1000, 2000, 0)
        
        self.right_motor = self.board.get_pin('d:3:s')
        self.board.servo_config(3, 1000, 2000, 0)
        
        self.enabled = False
        
        self.enabled_status = self.create_subscription(Bool, '/signals/enabled', self.enabled_handler,  10)
        self.enabled_status = self.create_subscription(Float32, '/motor/left', self.left_motor_control,  10)
        self.enabled_status = self.create_subscription(Float32, '/motor/right', self.right_motor_control,  10)

    def normalized_to_degrees(normalized_num: int):
        normalized_num = max(0, min(1.0, normalized_num))
        return normalized_num * 90

    def enabled_handler(self, msg: Bool):
        self.enabled = msg.data
        if self.enabled and not msg.data:
            # We need to shut off motors
            self.left_motor.write(self.normalized_to_degrees(0))
            self.right_motor.write(self.normalized_to_degrees(0))
        
    def left_motor_control(self, msg: Float32):
        if self.enabled:
            self.left_motor.write(self.normalized_to_degrees(msg.data))
    def right_motor_control(self, msg: Float32):
        if self.enabled:
            self.right_motor.write(self.normalized_to_degrees(msg.data))

def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    motor_node = Motor()
    
    rclpy.spin()

    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
