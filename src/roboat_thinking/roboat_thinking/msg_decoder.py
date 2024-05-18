from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
from roboat_interfaces.msg import RawData
from std_msgs.msg import Bool
import subprocess
import os
from bitstring import Bits, BitArray
from .pb.radio_comms_pb2 import ToBoatCommand, ToShoreResponse
from .pb.radio_comms_pb2 import EchoRequest

class MessageDecoder(Node):
    def __init__(self):
        super().__init__('radio')
     
        self.raw_radio_publisher = self.create_publisher(RawData, '/radio/raw_send', 10)
        self.subscription_ = self.create_subscription(RawData, '/radio/raw_recv', self.decode, 10)

        self.enabled_publisher = self.create_publisher(Bool, '/signals/enabled', 10)

        # self.timer = self.create_timer(timer_period_sec, self.read_from_radio) # Temperary Timer
    
    def decode(self, msg: RawData):
        self.get_logger().info(f"bytes: {msg.data}")
        recieved_msg = ToBoatCommand()
        try:
            recieved_msg.ParseFromString(b''.join(msg.data))
            msg_type = recieved_msg.WhichOneof("command")
            if msg_type == "echo":
                self.echo()
            elif msg_type == "control_status":
                enabled = recieved_msg.control_status.enabled
                set_control_status(True)
            else:
                self.get_logger().error(f"unhandled msg type: {msg_type}")
        except Exception as e:
            self.get_logger().error(f"Invalid Decoded Msg: {msg.data}, error: {e}")
    
    def set_control_status(self, enabled_status: bool):
        self.enabled_publisher.publish(enabled_status)
        self.get_logger().info(f"Set Control Status to: {enabled_status}")

    def echo(self):
        # Send Response
        msg_to_send = ToShoreResponse()
        msg_to_send.echo.SetInParent()
        encoded_bytes = msg_to_send.SerializeToString()
        print(encoded_bytes)
        print([encoded_bytes[i].to_bytes(1, 'big') for i in range(len(encoded_bytes))])
        self.raw_radio_publisher.publish(RawData(data=[encoded_bytes[i].to_bytes(1, 'big') for i in range(len(encoded_bytes))]))
       
        

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