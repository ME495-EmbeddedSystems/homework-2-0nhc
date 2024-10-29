#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

def generate_launch_description():
    # Arguments
    rviz_config = LaunchConfiguration("rviz_config")
    robot_description_path = LaunchConfiguration("robot_description_path")
    use_jsp = LaunchConfiguration("use_jsp")
    publish_freq = LaunchConfiguration("publish_freq")

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            "rviz_config",
            default_value="turtle_brick.rviz",
            description="Customized Rviz configuration file."
        ),
        DeclareLaunchArgument(
            "robot_description_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("turtle_brick"),
                "urdf",
                "turtle.urdf.xacro"
            ]),
            description="Path to the robot's xacro file."
        ),
        DeclareLaunchArgument(
            "use_jsp",
            default_value="gui",
            description="Different types of joint_state_publisher: gui/jsp/none"
        ),
        DeclareLaunchArgument(
            "publish_freq",
            default_value="100.0",
            description="Publish frequency."
        ),

        # Load the robot description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rsp",
            parameters=[{
                "robot_description": 
                    Command([ExecutableInPackage("xacro", "xacro"), 
                             " ",
                             robot_description_path]),
                "publish_frequency": publish_freq
            }]
        ),

        # Start Joint State Publisher
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'gui'"])),
            parameters=[{"rate": publish_freq}]
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'jsp'"])),
            parameters=[{"rate": publish_freq}]
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", PathJoinSubstitution([
                    FindPackageShare("turtle_brick"),
                    "rviz",
                    rviz_config
                ])
            ]
        )
    ])
