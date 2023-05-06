import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps

class GPS(Node):
    def __init__(self):
        super().__init__('gps')
        self.port = spidev.SpiDev()
        self.gps = UbloxGps(self.port)
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)

def main():
    print('Hi from roboat_perception.')


if __name__ == '__main__':
    main()
