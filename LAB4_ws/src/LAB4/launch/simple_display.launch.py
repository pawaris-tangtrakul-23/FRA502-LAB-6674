#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    # --- FIX: Changed 'example_description' to 'LAB4' ---
    # This ensures it looks for the 'config' and 'robot' folders inside YOUR package
    try:
        pkg = get_package_share_directory('LAB4')
    except Exception as e:
        print("Error: Could not find package 'LAB4'. Make sure you have built the package.")
        raise e

    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    
    # Check if file exists to prevent cryptic errors
    if not os.path.exists(path_description):
        print(f"Error: Xacro file not found at {path_description}")
    
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    control = Node(package="LAB4", executable="control.py", output='screen')
    random = Node(package="LAB4", executable="random_pose.py", output='screen')
    
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(control)
    launch_description.add_action(random)
    
    return launch_description