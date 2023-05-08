import rclpy
from rclpy.node import Node
import spidev
from ublox_gps import UbloxGps, core, sparkfun_predefines
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import subprocess
import os

class GPS(Node):
    def __init__(self):
        super().__init__('gps')
        self.port = spidev.SpiDev()
        self.gps = UbloxGps(self.port)

        
        timer_period = 2  # seconds
        self.gps_pos_timer = self.create_timer(timer_period, self.poll_gps)  

        timer_period = 60  # seconds
        self.gps_time_timer = self.create_timer(timer_period, self.set_time)    

        self.publisher_ = self.create_publisher(NavSatFix, '/sensor/gpsfix', 10)
        
        self.get_logger().info("GPS Initialized")
    
    def __del__(self):
        self.port.close()

    def poll_gps(self):
        try:
            geo = self.gps.geo_coords()
            cov = self._get_cov_gps();

            if geo.numSV == 0:
                self.get_logger().info("No GPS Signal", throttle_duration_sec=60)

            # From https://github.com/FrankBu0616/ros2_zed_f9r_gps/blob/main/zed_f9r_gps/zed_f9r_gps/publish_gps_data.py
            # and https://github.com/sparkfun/Qwiic_Ublox_Gps_Py/pull/26
            msg = NavSatFix()

            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"

            msg.longitude = geo.lon
            msg.latitude = geo.lat
            msg.altitude = 0.001 * float(geo.height)
            
            NN = cov.posCovNN
            NE = cov.posCovNE
            ND = cov.posCovND
            EE = cov.posCovEE
            ED = cov.posCovED
            DD = cov.posCovDD
            # Following the convention here: https://www.ros.org/reps/rep-0105.html
            msg.position_covariance = [EE, NE, ED, NE, NN, ND, ED, ND, DD]
            msg.position_covariance_type = 3

            self.publisher_.publish(msg)

            self.get_logger().info(f"Successfully published first gps_coords: {geo.lat}, {geo.lon} at {geo.gSpeed} mm/s", once=True)
        except (ValueError, IOError) as err:
            self.get_logger().error(f"GPS Error! {err}", throttle_duration_sec=60)
    
    def set_time(self):
        try:
            time_data = self.gps.date_time()

            # convert time_data to a form the date -u command will accept: "20140401 17:32:04"
            gps_utc = "{:04d}{:02d}{:02d} {:02d}:{:02d}:{:02d}".format(time_data.year, time_data.month, time_data.day,
                                                                       time_data.hour, time_data.min,
                                                                       time_data.sec)

            if not (time_data.valid.validDate == 1 and time_data.valid.validTime == 1):
                self.get_logger().warn(f"Time or Date is not Valid!\nvalidDate: {time_data.valid.validDate}\nvalidTime: {time_data.valid.validTime}", throttle_duration_sec=60)

            subprocess.run(["date", "-u", "--set={}".format(gps_utc)], timeout=2, stdout=subprocess.DEVNULL)
            self.get_logger().info(f"First Set Time to {gps_utc} UTC", once=True)
        except Exception as e:
            self.get_logger().error(f"GPS Error when setting time! {e}", throttle_duration_sec=60)
    
    def _get_cov_gps(self):
        """
        Sends a poll request for the NAV class with the COV Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and COV Message ID
        :rtype: namedtuple
        """
        self.gps.send_message(sparkfun_predefines.NAV_CLS, self.gps.nav_ms.get('COV'))
        parse_tool = core.Parser([sparkfun_predefines.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.gps.hard_port)

        return payload

        
def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS()

    rclpy.spin(gps_node)

    gps_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
