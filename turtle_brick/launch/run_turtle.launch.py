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
