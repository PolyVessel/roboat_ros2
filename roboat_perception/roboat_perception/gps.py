import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps
from roboat_interfaces.msg import GPSInfo

class GPS(Node):
    def __init__(self):
        super().__init__('gps')
        self.port = spidev.SpiDev()
        self.gps = UbloxGps(self.port)

        
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.poll_gps)    

        self.publisher_ = self.create_publisher(GPSInfo, '/sensor/gps_info', 10)
    
    def __del__(self):
        self.port.close()

    def poll_gps(self):
        try:
            geo = self.gps.geo_coords()
            if geo.numSV == 0:
                self.get_logger().info("No GPS Signal", throttle_duration_sec=60)

            gps_info = GPSInfo()
            gps_info.lon = geo.lon
            gps_info.lat = geo.lat
            gps_info.ground_speed = geo.gSpeed
            gps_info.header.stamp = self.get_clock().now().to_msg();

            self.publisher_.publish(gps_info)
        except (ValueError, IOError) as err:
            self.get_logger().error(f"GPS Error! {err}", throttle_duration_sec=60)
        
def main():
    gps_node = GPS()

    rclpy.spin(gps_node)

    gps_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
