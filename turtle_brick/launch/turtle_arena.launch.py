#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""Launch file to bringup the arena."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    # Paths to the included launch files
    show_turtle_launch = PathJoinSubstitution([
        FindPackageShare('turtle_brick'),
        'launch',
        'show_turtle.launch.py'
    ])

    run_turtle_launch = PathJoinSubstitution([
        FindPackageShare('turtle_brick'),
        'launch',
        'run_turtle.launch.py'
    ])

    # Return the launch description
    return LaunchDescription([
        # Include show_turtle.launch with the use_jsp argument set to "none"
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(show_turtle_launch),
            launch_arguments={'use_jsp': 'none'}.items(),
        ),

        # Include run_turtle.launch.xml
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_turtle_launch),
        ),

        # Launch the arena node
        Node(
            package='turtle_brick',
            executable='arena',
            name='arena',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('turtle_brick'),
                'config',
                'turtle.yaml'
            ])]
        ),

        # Launch the catcher node
        Node(
            package='turtle_brick',
            executable='catcher',
            name='catcher',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('turtle_brick'),
                'config',
                'turtle.yaml'
            ])]
        ),

        # Launch the static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0',
                       '0', '0', '0', '1',
                       'world', 'odom']
        )
    ])
