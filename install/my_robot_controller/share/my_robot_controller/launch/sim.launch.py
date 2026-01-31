import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Paths
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_controller = get_package_share_directory('my_robot_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_path_config = LaunchConfiguration('map_path')
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'my_map.world')
    nav2_params = os.path.join(pkg_controller, 'config', 'nav2_params.yaml')
    

    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(pkg_description, 'maps', 'my_3d_map.pcd'),
        description='Path to the PCD file'
    )

    map_loader = Node(
        package='my_robot_controller',
        executable='map_loader_node',
        parameters=[{
            'pcd_path': map_path_config,
            'frame_id': 'map'
        }]
    )

    # Static TF to link the map to your world origin
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    # 1. Robot State Publisher
    robot_description_raw = xacro.process_file(urdf_file).toxml()
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 4. 3D Map Accumulator (The "Perception" Module)
    map_3d_node = Node(
        package='my_robot_controller',
        executable='map_accumulator_3d',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )



    # 7. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        map_3d_node,
        rviz
    ])
