# add_two_ints_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    a_arg = DeclareLaunchArgument('a', default_value='10', description='First integer to add')
    b_arg = DeclareLaunchArgument('b', default_value='20', description='Second integer to add')

    add_service_node = Node(
        package='beginner_tutorials',
        executable='add_two_ints_service',
        name='add_two_ints_service'
    )

    add_client_node = Node(
        package='beginner_tutorials',
        executable='add_two_ints_client',
        name='add_two_ints_client',
        arguments=[LaunchConfiguration('a'), LaunchConfiguration('b')]
    )

    return LaunchDescription([a_arg, b_arg, add_service_node, add_client_node])
