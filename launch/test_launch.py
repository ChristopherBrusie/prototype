from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os

def generate_launch_description():
    main_nodes = [
        # motor driver
        Node(
            package='robocar',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # IMU node
        Node(
            package='robocar',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # teleop node
        Node(
            package='robocar', 
            executable='teleop_twist_keyboard', 
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # statis tf publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
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