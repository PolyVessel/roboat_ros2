from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class GPSSubscriber(Node):
    
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/sensor/gpsfix',
            self.listener_callback,
            10)
        self.get_logger().info('start up:')
        
        
    def listener_callback(self, msg: NavSatFix):
        coordinates = (msg.longitude, msg.latitude)
        alt = msg.altitude
        time = msg.header.stamp
        direction = msg.position_covariance
        
        self.get_logger().info('coordinates: "%s"' % coordinates)
        self.get_logger().info('altitude: "%s"' % alt)
        self.get_logger().info('time stamp: "%s"' % time)
        self.get_logger().info('direction: "%s"' % direction)
        
        
def main(args: List[str] | None = None):
    rclpy.init(args=args)
    
    gps_subscriber = GPSSubscriber()
    
    rclpy.spin(gps_subscriber)
    
    gps_subscriber.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == '__main__':
    main()
        