import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # ===============================
    # Package paths
    # ===============================
    pkg_controller = get_package_share_directory('my_robot_controller')
    pkg_description = get_package_share_directory('my_robot_description')

    # ===============================
    # Files
    # ===============================
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'my_map.world')
    
    # FORCE THE MAP PATH DIRECTLY
    #pcd_file = os.path.join(pkg_description, 'maps', 'my_3d_map.pcd')

    # ===============================
    # Robot description
    # ===============================
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ===============================
    # Gazebo
    # ===============================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ],
        output='screen'
    )

    # ===============================
    # 3D Map Loader (HARDCODED DEBUG MODE)
    # ===============================
    # DELETE THIS LINE: pcd_file = os.path.join(pkg_description, 'maps', 'my_3d_map.pcd')
    
    # USE THIS INSTEAD (Paste the path from Step 1):
    pcd_file = '/root/ros2_ws/vs/src/my_robot_description/maps/my_3d_map.pcd'

    dynamic_3d_map_manager = Node(
        package='my_robot_controller',
        executable='dynamic_3d_map_manager',
        name='dynamic_3d_map_manager',
        output='screen',
        parameters=[{
            'pcd_file_path': pcd_file,
            'map_frame_id': 'map'
        }]
    )
    # ===============================
    # Static TF Publisher (CRITICAL)
    # ===============================
    # If the map manager doesn't publish map->odom, this will force it.
    # Args: x y z yaw pitch roll parent child
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # ===============================
    # Other Nodes
    # ===============================
    obstacle_detector = Node(
        package='my_robot_controller',
        executable='obstacle_detector_3d',
        name='obstacle_detector_3d',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    costmap_2d = Node(
        package='my_robot_controller',
        executable='costmap_2d_from_3d',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    a_star_planner = Node(
        package='my_robot_controller',
        executable='a_star_planner',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # ===============================
    # Local Planner (Potential Fields)
    # ===============================
    local_planner = Node(
        package='my_robot_controller',
        executable='local_planner',
        name='local_planner',
        output='screen'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        map_to_odom, # Added this to ensure map frame exists even if loader is slow
        robot_state_publisher,
        gazebo,
        spawn_robot,
        dynamic_3d_map_manager,
        obstacle_detector,
        costmap_2d,
        a_star_planner,
        local_planner,
        rviz
    ])
