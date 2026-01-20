import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_controller = get_package_share_directory('my_robot_controller')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'my_map.world')

    # 2. Process URDF
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 4. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 5. Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 6. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
        # Optional: arguments=['-d', rviz_config_path]
    )

    # 7. Lidar Listener (Safety/AEB Node)
    # This node monitors 3D points and publishes stop commands to cmd_vel [cite: 11, 16, 19]
    lidar_safety_node = Node(
        package='my_robot_controller',
        executable='lidar_listener',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz,
        lidar_safety_node
    ])
