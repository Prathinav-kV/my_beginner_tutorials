# Copyright 2024 Prathinav Karnala Venkata
"""
@file service_launch.py
@brief Launch file to start the talker and listener nodes with configurable parameters.
 
This launch file initializes both the talker and listener nodes. The talker node's 
frequency can be modified via command-line arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate the launch description to start talker and listener nodes.

    @return LaunchDescription Launch description object containing all the nodes and arguments.
    """
    
    # Declare a launch argument for the publish frequency
    frequency_arg = DeclareLaunchArgument(
        'frequency', default_value='2.0',
        description='Frequency for the talker node on /chatter topic (in Hz)'
    )

    # Use LaunchConfiguration to fetch the frequency argument value
    frequency = LaunchConfiguration('frequency')

    # Define the talker node with a parameter for frequency
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        parameters=[{'frequency': frequency}]
    )

    # Define the listener node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='listener'
    )

    # Create the launch description with the nodes and the argument
    return LaunchDescription([
        frequency_arg,
        talker_node,
        listener_node
    ])
