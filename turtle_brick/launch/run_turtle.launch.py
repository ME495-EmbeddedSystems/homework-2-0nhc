#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Turtlesim
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            output="screen",
            parameters=[{"holonomic": True}]
        ),
        
        # Launch Turtle Robot
        Node(
            package="turtle_brick",
            executable="turtle_robot",
            name="turtle_robot",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("turtle_brick"),
                "config",
                "turtle.yaml"
            ])]
        ),
    ])
