#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


import serial
import time
import math


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info('Motor Driver Node has been started.')

        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Twist, 'wheel_odometry', 10)


        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            self.ser = None

        time.sleep(2)  # Wait for ESP32 to reset

        # wheel stuff
        self.rpm_factor = 60 / (math.pi * 0.06) # wheel diameter 0.06m
        self.L = 20 #distance from center to wheel in X (half robot width)
        self.W = 20 #distance from center to wheel in Y (half robot length)
        self.L_W = self.L + self.W

        self.create_timer(0.05, self.read_serial_callback) # 20hz

    def read_serial_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                try:
                    rpms = list(map(float, line.split()))
                    if len(rpms) == 4:
                        msg = Twist()
                        msg.linear.x = (rpms[0] + rpms[1] + rpms[2] + rpms[3]) / 4.0 / self.rpm_factor
                        msg.linear.y = (rpms[1] - rpms[0] + rpms[3] - rpms[2]) / 4.0 / self.rpm_factor
                        msg.angular.z = (rpms[2] - rpms[3] + rpms[1] - rpms[0]) / (4.0 * self.L_W) / self.rpm_factor
                        self.odom_pub.publish(msg)
                except ValueError as e:
                    self.get_logger().error(f"Failed to parse RPMs: {e}")



    def cmd_vel_callback(self, msg):
        # motor order, direction 
        #       ^
        # 0   O| |O   1
        # 2   O| |O   3
        # 
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        lpw = self.L_W * wz
        rpm_factor = self.rpm_factor
        
        rpms = [(vx - vy - lpw) * rpm_factor,
            (vx + vy + lpw) * rpm_factor,
            (vx + vy - lpw) * rpm_factor,
            (vx - vy + lpw) * rpm_factor]
    
        self.ser.write(f"{rpms[0]} {rpms[1]} {rpms[2]} {rpms[3]}\n".encode())



    # cmd = f"{int(w0)} {int(w1)} {int(w2)} {int(w3)}\n"
    # try:
    #     self.ser.write(cmd.encode())
    #     self.get_logger().info(f"Sent to ESP32: {cmd.strip()}")
    # except Exception as e:
    #     self.get_logger().error(f"Serial write failed: {e}")



def destory_node(self):
    if self.ser:
        self.ser.close()
    super().destroy_node()




def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



