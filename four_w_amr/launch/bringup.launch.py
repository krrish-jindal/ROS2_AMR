#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Constants for paths to different files and folders
    package_name = 'four_w_amr'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Declare the transport_type argument
    transport_type = DeclareLaunchArgument('transport_type', default_value='serial', description='Transport type (e.g., udp4, udp6, tcp4, tcp6, canfd, serial, multiserial, pseudoterminal)')
    # Declare the serial device argument
    serial_dev = DeclareLaunchArgument('serial_dev', default_value='/dev/ttyMyDevice', description='Serial device')
    # Declare the baudrate argument
    baudrate = DeclareLaunchArgument('baudrate', default_value='115200', description='Baudrate for serial communication')


    start_twist_to_pwm = Node(
        package='four_w_amr',
        executable='twist_2_pwm.py',
        name='twist_2_pwm',
        output='both')
    
    start_odom_publisher = Node(
        package='four_w_amr',
        executable='odom.py',
        name='odom_pub',
        output='both')
    
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        output='screen',
        arguments=[LaunchConfiguration('transport_type'), '--dev', LaunchConfiguration('serial_dev'), '-b', LaunchConfiguration('baudrate')],  # Include the --dev and -b/--baudrate arguments, and -h/--help option
        respawn=True)

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Add the launch arguments to the LaunchDescription
    ld.add_action(transport_type)
    ld.add_action(serial_dev)
    ld.add_action(baudrate)

    # Add the nodes to the LaunchDescription
    ld.add_action(start_twist_to_pwm)
    ld.add_action(start_odom_publisher)
    ld.add_action(micro_ros_agent_node)

    return ld
