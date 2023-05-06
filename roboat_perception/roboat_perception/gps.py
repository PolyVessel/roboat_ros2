import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps
from roboat_interfaces.msg import GPSInfo
import subprocess
import os

class GPS(Node):
    def __init__(self):
        super().__init__('gps')
        self.port = spidev.SpiDev()
        self.gps = UbloxGps(self.port)

        
        timer_period = 2  # seconds
        self.gps_pos_timer = self.create_timer(timer_period, self.poll_gps)  

        timer_period = 2  # seconds
        self.gps_time_timer = self.create_timer(timer_period, self.set_time)    

        self.publisher_ = self.create_publisher(GPSInfo, '/sensor/gps_info', 10)
        
        self.get_logger().info("GPS Initialized")
    
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
    
    def set_time(self):
        try:
            time_data = self.gps.date_time()

            # convert time_data to a form the date -u command will accept: "20140401 17:32:04"
            gps_utc = "{:04d}{:02d}{:02d} {:02d}:{:02d}:{:02d}".format(time_data.year, time_data.month, time_data.day,
                                                                       time_data.hour, time_data.min,
                                                                       time_data.sec)

            if time_data.valid.validDate != True or time_data.valid.validTime != True:
                self.get_logger().warn(f"Time or Date is not Valid!\nvalidDate: {time_data.valid.validDate}\nvalidTime: {time_data.valid.validTime}", throttle_duration_sec=60)

            self.get_logger().info(f"validDate: {time_data.valid.validDate}\nvalidTime: {time_data.valid.validTime}", throttle_duration_sec=60)
            subprocess.run(["date", "-u", "--set={}".format(gps_utc)], timeout=2)
        except Exception as e:
            self.get_logger().error(f"GPS Error when setting time! {e}", throttle_duration_sec=60)

        
def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS()

    rclpy.spin(gps_node)

    gps_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
