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
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # IMU node
        Node(
            package='prototype',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # IMU Madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'use_mag': False},
                        {'remove_gravity_vector': True},
                        {'publish_tf': False},
                        {'gain':0.1},
                        {'zeta: 0.0'},
                        ],
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


    #static transforms
    config_file = os.path.join(
        get_package_share_directory('prototype'),
        'launch',
        'static_tf.yaml')

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    static_tf_nodes = []
    for tf in config['transforms']:
        x, y, z = tf['translation']
        roll, pitch, yaw = tf['rotation']
        static_tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f"static_tf_{tf['name']}",
                arguments=[
                    str(x), str(y), str(z),
                    str(roll), str(pitch), str(yaw),
                    tf['parent'], tf['child']
                ]
            )
        )

    return LaunchDescription(static_tf_nodes + main_nodes)