import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np
import math



class MutliStaticTFPublisher(Node):
    def init(self):
        super().__init__('multi_static_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        self.publish_transforms()

    def publish_transforms(self):
        transforms = []
        t1 = TransformStamped()
        t1.header.frame_id = 'base_link'
        t1.header.child_frame_id = 'imu_link'
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.transform.translation.x = 0.1
        t1.transform.translation.y = 0.1
        t1.transform.translation.z = 0.0
        q1 = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]
        transforms.append(t1)

        t2 = TransformStamped()
        t2.header.frame_id = 'base_link'
        t2.header.child_frame_id = 'lidar_link'
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.2
        q2 = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]
        transforms.append(t2)
        
        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MutliStaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

