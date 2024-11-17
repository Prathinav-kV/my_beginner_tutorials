import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate the launch description for the talker and listener nodes, and optionally record a bag file.

    @details
    - Includes a launch argument `record` to enable or disable bag recording.
    - Starts the talker and listener nodes.
    - Records all topics to a bag file in the `results/bag_record` directory if `record` is enabled.
    - Stops the recording automatically after 15 seconds.

    @return LaunchDescription containing nodes, arguments, and processes for the launch file.
    """
    # Declare argument for enabling/disabling recording
    record_enabled = LaunchConfiguration('record', default='true')
    bag_directory = os.path.join(os.getcwd(), 'results', 'bag_record')

    return LaunchDescription([
        # Launch argument for enabling/disabling recording
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Enable or disable bag recording'
        ),

        # Talker node (publisher)
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            parameters=[{'frequency': 2.0}],
            output='screen'
        ),

        # Listener node (subscriber)
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener',
            output='screen'
        ),

        # Conditional bag recording
        TimerAction(
            period=0.0,  # Start immediately
            actions=[
                ExecuteProcess(
                    condition=IfCondition(record_enabled),
                    cmd=[
                        'ros2', 'bag', 'record', '-a', '-o', bag_directory
                    ],
                    output='screen'
                )
            ]
        ),

        # Timer to stop the recording after ~15 seconds
        TimerAction(
            period=15.0,  # Stop after 15 seconds
            actions=[
                ExecuteProcess(
                    condition=IfCondition(record_enabled),
                    cmd=['killall', '-SIGINT', 'ros2'],
                    output='screen'
                )
            ]
        ),
    ])
