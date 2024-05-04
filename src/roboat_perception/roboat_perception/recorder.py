from datetime import datetime
from time import sleep
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import NavSatFix
from csv import writer
import os as os
from glob import glob 
import sqlite3


class Recorder(Node):
    def __init__(self, path):
        super().__init__('recorder')
        self.subscription = self.create_subscription(NavSatFix, '/sensor/gpsfix', self.gps_callback, 10)
        self.con = sqlite3.connect(path)
        cur = self.con.cursor()
        cur.execute("""
            CREATE TABLE IF NOT EXISTS  "GPS" (
                "Lat" REAL NOT NULL,
                "Lon" REAL NOT NULL,
                "Timestamp" INTEGER NOT NULL
            ) STRICT;
        """)
        self.con.commit()
        
        self.get_logger().info("Recorder successfully initialized!")

    def gps_callback(self, getMsg: NavSatFix):
        print(getMsg.latitude, getMsg.longitude, getMsg.header.stamp.sec)
        
        self.con.execute("INSERT INTO GPS(Lat, Lon, Timestamp) VALUES (?, ?, ?)",
            (getMsg.latitude, getMsg.longitude, getMsg.header.stamp.sec))
        self.con.commit()

        
def main():
    rclpy.init()
    node = Recorder('/tmp/recording.db') # TODO: Change me so we persist data, this is to protect SD card
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()