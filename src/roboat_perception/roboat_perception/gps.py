from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps, core, sparkfun_predefines
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import subprocess
import os

class GPS(Node):
    def __init__(self) -> None:
        super().__init__('gps')
        self.port = spidev.SpiDev()
        self.gps = UbloxGps(self.port)

        self.publisher_ = self.create_publisher(NavSatFix, '/sensor/gpsfix', 10)
        
        self.get_logger().info("GPS Initialized")
    
    def publish(self) -> None:
        try:
            while(True):
                geo = self.gps.geo_coords()

                if geo.numSV == 0:
                    self.get_logger().info("No GPS Signal", throttle_duration_sec=60)
                
                # From https://github.com/FrankBu0616/ros2_zed_f9r_gps/blob/main/zed_f9r_gps/zed_f9r_gps/publish_gps_data.py
                msg = NavSatFix()

                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gps"

                msg.longitude = geo.lon
                msg.latitude = geo.lat
                msg.altitude = 0.001 * float(geo.height)
             
                msg.position_covariance_type = 0

                self.publisher_.publish(msg)

                self.get_logger().info(f"Successfully published first gps_coords: {geo.lat}, {geo.lon} at {geo.gSpeed} mm/s", once=True)
        except (ValueError, IOError) as err:
            self.get_logger().error(f"GPS Error! {err}", throttle_duration_sec=60)
        finally:
            self.port.close()

def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    gps_node = GPS()
    gps_node.publish()

    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
