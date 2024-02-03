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
            quaternion = self.sensor.quaternion()
            ang_velocity = self.sensor.angular_velocity()
            lin_acceleration = self.sensor.linear_acceleration()

            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu"

            msg.orientation_covariance[0] = -1

            quat = Quaternion()
            quat.x = quaternion.x
            quat.y = quaternion.y
            quat.z = quaternion.z
            quat.w = quaternion.w

            ang_vel = Vector3()
            ang_vel.x = ang_velocity.x
            ang_vel.y = ang_velocity.y
            ang_vel.z = ang_velocity.z

            lin_acc = Vector3()
            lin_acc.x = lin_acceleration.x
            lin_acc.y = lin_acceleration.y
            lin_acc.z = lin_acceleration.z

            msg.quaternion = quat
            msg.angular_velocity = ang_vel
            msg.linear_acceleration = lin_acc

            self.publisher_.publish(msg)
            
            self.get_logger().info(f"Successfully published first IMU Quaternion -> x: 
                                    {quaternion.x}, y: {quaternion.y}, z: {quaternion.z}, w: {quaternion.w}", once = True)

        except (ValueError, IOError) as err:
            self.get_logger().error(f"IMU Error! {err}")


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    imu_node = IMU()

    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()