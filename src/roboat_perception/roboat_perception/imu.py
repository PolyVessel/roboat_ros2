from typing import List, NamedTuple
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import adafruit_bno055
import board

GET_IMU_PERIOD = 2

class IMU(Node):
    def __init__(self) -> None:
        super().__init__('imu')
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        self.get_logger().info("IMU Initialized")
        
        self.timer = self.create_timer(GET_IMU_PERIOD, self.publish)


    def publish(self) -> None:
        try:
            (qx, qy, qz, qw) = self.sensor.quaternion
            (gx, gy, gz) = self.sensor.gyro
            (lax, lay, laz) = self.sensor.linear_acceleration

            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu"

            msg.orientation_covariance[0] = -1

            quat = Quaternion()
            quat.x = qx
            quat.y = qy
            quat.z = qz
            quat.w = qw

            gyro = Vector3()
            gyro.x = gx
            gyro.y = gy
            gyro.z = gz

            lin_acc = Vector3()
            lin_acc.x = lax
            lin_acc.y = lay
            lin_acc.z = laz

            msg.quaternion = quat
            msg.angular_velocity = gyro
            msg.linear_acceleration = lin_acc

            self.publisher_.publish(msg)
            
            self.get_logger().info(f"Successfully published first IMU Quaternion -> x: {qx}, y: {qy}, z: {qz}, w: {qw}", once = True)

        except (ValueError, IOError) as err:
            self.get_logger().error(f"IMU Error! {err}")


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    imu_node = IMU()
    
    try:
        rclpy.spin(imu_node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')    

    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()