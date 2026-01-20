from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='map_accumulator_3d',
            name='map_3d_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
