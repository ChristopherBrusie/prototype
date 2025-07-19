from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    main_nodes = [
        # motor driver
        Node(
            package='prototype',
            executable='motor_driver_node',
            name='motor_driver_node',
            #output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Static TF
        Node(
            package='prototype',
            executable='multi_static_tf_publisher',
            name='multi_static_tf_publisher',
            #output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # IMU node
        Node(
            package='prototype',
            executable='imu_node',
            name='imu_node',
            #output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # IMU Madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter',
            #output='screen',
            parameters=[os.path.join(
                get_package_share_directory('prototype'),
                'launch',
                'madgwick_config.yaml')], 
            remappings=[
                ('/imu/data', '/imu/data_madgwick')
            ]
        ), 

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('prototype'),
                'launch',
                'ekf_config.yaml')],            
            remappings=[
                ('/imu/data', '/imu/data_raw'),
                ('/wheel_encoder_velocity', '/wheel_odometry')
            ]
        )
    ]


    
    return LaunchDescription(main_nodes)