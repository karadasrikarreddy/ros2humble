import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Path to your URDF (Make sure the package name matches)
    pkg_path = os.path.join(get_package_share_directory('my_robot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Robot State Publisher (Publishes the 3D model)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 3. Gazebo (Starts the empty world)
    # Inside your sim.launch.py, update the 'gazebo' section:

    # 1. Get the path to the world file
    world_file_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'worlds',
        'my_map.world'
    )

    # 2. Update the Gazebo include to use this world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # 4. Spawn Entity (Put the robot in the Gazebo world)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 5. Your C++ Drive Node
   # drive_node = Node(
    #    package='my_robot_controller',
     #   executable='drive_node',
      #  output='screen'
    #) 

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
     #   drive_node
    ])
