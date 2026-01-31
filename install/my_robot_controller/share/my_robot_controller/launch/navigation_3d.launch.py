import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('my_robot_description')
    
    # Path to your saved 3D map (PCD file)
    # Note: For now, we will stream the live cloud into Octomap 
    # as it's easier to debug in simulation.
    
    return LaunchDescription([
        # 1. OCTOMAP SERVER: Turns 3D points into a Navigation Map
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05, # 5cm cubes
                'frame_id': 'odom', # Global frame
                'sensor_model.max_range': 10.0,
                'save_free_cells': True # Important for planning
            }],
            remappings=[('cloud_in', '/global_map_3d')]
        ),

        # 2. NAV2 BRINGUP: The "Brains" of the movement
        # We will use the default Nav2 config to start
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 3. NAV2 PLANNER & CONTROLLER
        # (Simplified for the first 3D test)
    ])
