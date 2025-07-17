from launch import LaunchDescription
from launch_ros.actions import Node
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
                        {'gain': 0.1},
                        {'remove_gravity_vector': True},
                        ],
            remappings=[
                ('/imu/data', '/imu/filtered')
            ]
        )
    ]


    #static transforms
    config_file = os.path.join(os.path.dirname(__file__), 'static_tf.yaml')
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