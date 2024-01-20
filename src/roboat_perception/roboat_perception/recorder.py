from datetime import datetime
import uuid
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
import uuid
import glob 


class Recorder(Node):
    def __init__(self, directory):
        super().__init__('recorder')
        self.subscription = self.create_subscription(NavSatFix, '/sensor/gpsfix', self.gps_callback, 10)
        self.directory = directory

    def gps_callback(self, getMsg: NavSatFix):
        coordinates = (getMsg.latitude, getMsg.longitude)
        timestamp = getMsg.header.stamp
        filename = self.generate_unique_filename()
        with open(os.path.join(self.directory, filename), 'a+') as collect_info:
            writer_object = writer(collect_info)
            writer_object.writerow([timestamp, coordinates[0], coordinates[1]])
            
        self.delete_oldest_file()

        # if os.stat('~/gps_log.csv').st_size > 5_000_000:  
        #      f = open("newinfo.txt", "w")
            # f.write(f" {timestamp}, {coordinates[0]}, {coordinates[1]}\n")
            
        
        

    def generate_unique_filename(self):
        # unique_id = str(uuid.uuid4())
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
        filename = f"{timestamp}.csv"
        return os.path.join(self.directory, filename)
                
   
    def delete_oldest_file(self):
        # Get a list of files matching the naming pattern in the specified directory
        files = glob(os.path.join(self.directory, '*.csv'))

        # Sort files based on creation time
        sorted_files = sorted(files, key=lambda x: os.path.getctime(x))

        # Delete the oldest file if exceeding a certain condition (e.g., more than 5 files)
        while len(sorted_files) > 5:
            file_to_delete = sorted_files.pop(0)
            os.remove(file_to_delete)

        
        
        
        
        
        
def main():
    rclpy.init()
    node = Recorder('~/logs')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()