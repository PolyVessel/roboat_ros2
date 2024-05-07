from time import sleep
from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
from roboat_interfaces.msg import RawData
import subprocess
import os
from bitstring import Bits, BitArray
from .pb.radio_comms_pb2 import ToBoatCommand, ToShoreResponse
from .pb.radio_comms_pb2 import EchoRequest

class MessageDecoder(Node):
    def __init__(self):
        super().__init__('radio')
     
        self.publisher_ = self.create_publisher(RawData, '/radio/raw_send', 10)
        self.subscription_ = self.create_subscription(RawData, '/radio/raw_recv', self.decode, 10)

        # self.timer = self.create_timer(timer_period_sec, self.read_from_radio) # Temperary Timer
    
    def decode(self, msg: RawData):
        self.get_logger().info(f"bytes: {msg.data}")
        recieved_msg = ToBoatCommand()
        try:
            recieved_msg.ParseFromString(b''.join(msg.data))
            msg_type = recieved_msg.WhichOneof("command")
            if msg_type == "echo":
                self.echo()
            else:
                self.get_logger().error(f"unhandled msg type: {msg_type}")
        except:
            self.get_logger().error(f"Invalid Decoded Msg: {msg.data}")
        

    def echo(self):
        # Send Response
        msg_to_send = ToShoreResponse()
        msg_to_send.response.echo.SetInParent()
        encoded_bytes = msg_to_send.SerializeToString()
        self.publisher_.publish([encoded_bytes[i].to_bytes(1, 'big') for i in range(len(encoded_bytes))])
        

def main(args=None):
    rclpy.init(args=args)

    radio = MessageDecoder()
    
    try:
        rclpy.spin(radio)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
