#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    
    # Package name - replace with your actual package name
    pkg_name = 'my_bot'
    
    # Paths to files
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    urdf_dir = os.path.join(pkg_share, 'description')
    urdf_file = os.path.join(urdf_dir, 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_urdf = robot_description_config.toxml()
    
    controller_config_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add this AFTER the robot_state_publisher_node definition
    '''
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            controller_config_file,
            {'use_sim_time': use_sim_time}
    ]
    )
'''

    # Mecanum Drive Controller spawner
    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'verbose': 'false'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node) 
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(mecanum_drive_spawner)

    return ld