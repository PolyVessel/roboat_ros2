import serial
import RPi.GPIO as GPIO
from time import sleep
from typing import List, NamedTuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from sensor_msgs.msg import NavSatFix
from csv import writer
import os as os


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        self.subscription = self.create_subscription(NavSatFix, '/sensor/gpsfix', self.gps_callback, 10)


    def gps_callback(self, getMsg):
        coordinates = (getMsg.latitude, getMsg.longitude)
        timestamp = getMsg.header.stamp
        with open('~/gps_log.csv', 'a') as collect_info:
                writer_object = writer(collect_info)
                writer_object.writerow([timestamp, coordinates[0], coordinates[1]])
        if os.stat('~/gps_log.csv').st_size > 5_000_000:  
            f = open("newinfo.txt", "w")
            f.write(f" {timestamp}, {coordinates[0]}, {coordinates[1]}\n")
            
        
        

            
                


        
        
        
        
        
        
def main():
     rclpy.init(args=args)
     recorder = Recorder()
        