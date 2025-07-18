import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import smbus

# Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43




class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.get_logger().info('IMU Node has been started.')

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50hz imu message

        # Initialize I2C bus
        self.bus = smbus.SMBus(1)
        self.init_mpu6050()

    def init_mpu6050(self):
        # Wake up MPU6050
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(MPU6050_ADDR, addr + 1)
        value = (high << 8) + low
        if value > 32768:
            value -= 65536
        return value



    def publish_imu_data(self):
        imu_msg = Imu()
        
        # Read accelerometer data
        accel_x = self.read_raw_data(ACCEL_XOUT_H) / 16384.0 * 9.80665
        accel_y = self.read_raw_data(ACCEL_XOUT_H + 2) / 16384.0 * 9.80665
        accel_z = self.read_raw_data(ACCEL_XOUT_H + 4) / 16384.0 * 9.80665
        
        # Read gyroscope data
        gyro_x = self.read_raw_data(GYRO_XOUT_H) / 131.0 * 0.0174533
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2) / 131.0 * 0.0174533
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4) / 131.0 * 0.0174533
        
        # Fill IMU message
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # pretty much junk but maybe useful later
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1.0  # Marks orientation as not available
        imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]  # tune based on noise
        imu_msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # Or whatever frame your URDF defines
        # Publish the IMU message
        self.imu_pub.publish(imu_msg)





def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()