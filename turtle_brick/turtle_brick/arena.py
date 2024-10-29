#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""The arena ROS 2 node for hopmework-2.

The arena node communicates through several ROS 2 protocols:

PUBLISHERS:
  + boundaries (visualization_msgs.msg.MarkerArray) - The boundaries of the
    arena
  + brick (visualization_msgs.msg.Marker) - The brick in the arena
  + arena_state (std_msgs.msg.Float32) - The state of the arena

SUBSCRIBERS:
  + turtle1/pose (turtlesim.msg.Pose) - The pose state of the turtle
  + tilt (turtle_brick_interfaces.msg.Tilt) - The platform's rotating angle

SERVICES:
  + place (turtle_brick_interfaces.srv.Place) - To place the brick on the
    platform
  + drop (std_srvs.srv.Empty) - To drop the brick from the platform

PARAMETERS:
  + frequency (double) - Timer frequency for the main loop
  + rainbow_frequency (double) - Timer frequency to control the rainbow effect
  + physics_frequency (double) - Timer frequency to control the physics of the
    brick
  + num_markers_per_side (int) - Number of markers per side of the arena
  + boundary_width (double) - Width of the boundaries
  + arena_width (double) - Width of the arena
  + arena_height (double) - Height of the arena
  + brick_size_x (double) - Size of the brick in the x-axis
  + brick_size_y (double) - Size of the brick in the y-axis
  + brick_size_z (double) - Size of the brick in the z-axis
  + gravity (double) - Gravity acceleration
  + platform_height (double) - Height of the platform
  + friction_dx (double) - Friction coefficient
  + platform_cylinder_radius (double) - Radius of the platform cylinder
  + robot_name (string) - Name of the robot
"""
import math
import time

from geometry_msgs.msg import Point, PoseStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32

from std_srvs.srv import Empty

import tf2_ros

import tf_transformations

from turtle_brick.physics import World
from turtle_brick.states import DROPPED, DROPPING, FALLING, INIT, PLACED
from turtle_brick.states import SLIDING

from turtle_brick_interfaces.msg import Tilt
from turtle_brick_interfaces.srv import Place

from turtlesim.msg import Pose

from visualization_msgs.msg import Marker, MarkerArray


class ArenaNode(Node):
    """The arena ROS 2 node for hopmework-2.

    The arena node communicates through several ROS 2 protocols:

    PUBLISHERS:
    + boundaries (visualization_msgs.msg.MarkerArray) - The boundaries of the
      arena
    + brick (visualization_msgs.msg.Marker) - The brick in the arena
    + arena_state (std_msgs.msg.Float32) - The state of the arena

    SUBSCRIBERS:
    + turtle1/pose (turtlesim.msg.Pose) - The pose state of the turtle
    + tilt (turtle_brick_interfaces.msg.Tilt) - The platform's rotating angle

    SERVICES:
    + place (turtle_brick_interfaces.srv.Place) - To place the brick on the
      platform
    + drop (std_srvs.srv.Empty) - To drop the brick from the platform

    PARAMETERS:
    + frequency (double) - Timer frequency for the main loop
    + rainbow_frequency (double) - Timer frequency to control the rainbow
      effect
    + physics_frequency (double) - Timer frequency to control the physics of
      the brick
    + num_markers_per_side (int) - Number of markers per side of the arena
    + boundary_width (double) - Width of the boundaries
    + arena_width (double) - Width of the arena
    + arena_height (double) - Height of the arena
    + brick_size_x (double) - Size of the brick in the x-axis
    + brick_size_y (double) - Size of the brick in the y-axis
    + brick_size_z (double) - Size of the brick in the z-axis
    + gravity (double) - Gravity acceleration
    + platform_height (double) - Height of the platform
    + friction_dx (double) - Friction coefficient
    + platform_cylinder_radius (double) - Radius of the platform cylinder
    + robot_name (string) - Name of the robot
    """

    def __init__(self):
        """Initialize the ArenaNode with configurable ROS parameters.

        Sets up ROS 2 parameters, initializes publishers, subscribers,
        services, timers, and physics-related attributes for the arena
        environment.
        """
        super().__init__('arena')
        # Declare timer frequency parameter, default to 100 Hz
        self.declare_parameter('frequency', 100.0)
        self._timer_frequency = self.get_parameter(
            'frequency').get_parameter_value().double_value
        # Declare rainbow frequency parameter, default to 100 Hz
        self.declare_parameter('rainbow_frequency', 100.0)
        self._rainbow_frequency = self.get_parameter(
            'rainbow_frequency').get_parameter_value().double_value
        # Declare physics frequency parameter, default to 250 Hz
        self.declare_parameter('physics_frequency', 250.0)
        self._physics_frequency = self.get_parameter(
            'physics_frequency').get_parameter_value().double_value
        # Declare number of markers per side parameter, default to 20
        self.declare_parameter('num_markers_per_side', 50)
        self._num_markers_per_side = self.get_parameter(
            'num_markers_per_side').get_parameter_value().integer_value
        # Declare boundary width parameter, default to 0.3
        self.declare_parameter('boundary_width', 0.3)
        self._boundary_width = self.get_parameter(
            'boundary_width').get_parameter_value().double_value
        # Declare arena width parameter, default to 12.0
        self.declare_parameter('arena_width', 12.0)
        self._arena_width = self.get_parameter(
            'arena_width').get_parameter_value().double_value
        # Declare arena height parameter, default to 0.8
        self.declare_parameter('arena_height', 0.8)
        self._arena_height = self.get_parameter(
            'arena_height').get_parameter_value().double_value
        # Declare brick size x parameter, default to 0.5
        self.declare_parameter('brick_size_x', 0.5)
        self._brick_size_x = self.get_parameter(
            'brick_size_x').get_parameter_value().double_value
        # Declare brick size y parameter, default to 0.4
        self.declare_parameter('brick_size_y', 0.4)
        self._brick_size_y = self.get_parameter(
            'brick_size_y').get_parameter_value().double_value
        # Declare brick size z parameter, default to 0.3
        self.declare_parameter('brick_size_z', 0.3)
        self._brick_size_z = self.get_parameter(
            'brick_size_z').get_parameter_value().double_value
        # Declare gravity acceleration parameter, default to -9.81
        self.declare_parameter('gravity', -9.81)
        self._gravity = self.get_parameter(
            'gravity').get_parameter_value().double_value
        # Declare robot urdf parameter, default to 1.0
        self.declare_parameter('platform_height', 1.0)
        self._platform_height = self.get_parameter(
            'platform_height').get_parameter_value().double_value
        # Declare friction_dx parameter, default to 0.004
        self.declare_parameter('friction_dx', 0.004)
        self._friction_dx = self.get_parameter(
            'friction_dx').get_parameter_value().double_value
        self.declare_parameter('platform_cylinder_radius', 0.1)
        self._platform_cylinder_radius = self.get_parameter(
            'platform_cylinder_radius').get_parameter_value().double_value
        self._tolerance = self._platform_cylinder_radius/2
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value
        # Remove '/'
        while (self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]

        # Begin_Citation [1] #
        """
        We use TRANSIENT_LOCAL durability for the publisher.
        By setting both publisher and subscriber
        Durability to TRANSIENT_LOCAL we emulate the effect of 'latched
        publishers' from ROS 1 (See https://github.com/ros2/ros2/issues/464)
        Essentially this means that when subscribers first connect to the
        topic they receive the last message published on the topic. Useful for
        example because rviz might open after the initial markers are
        published.
        """
        markerQoS = QoSProfile(depth=10,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # End_Citation [1] #

        # Setup Main Loop Timer
        self._main_loop_timer = self.create_timer(
            1.0/self._timer_frequency,
            self._main_loop_timer_callback)

        # Setup Rainbow Timer
        self._rainbow_timer = self.create_timer(
            1.0/self._rainbow_frequency,
            self._rainbow_timer_callback)
        self._start_time = time.time()
        self._num_markers_in_boundaries = 4*self._num_markers_per_side

        # Setup Physics Timer
        self._physics_timer = self.create_timer(
            1.0/self._physics_frequency,
            self._physics_timer_callback)

        # Setup Boundaries Publisher
        self._boundaries_publisher = self.create_publisher(MarkerArray,
                                                           '/boundaries',
                                                           markerQoS)
        self._boundaries = MarkerArray()
        self._boundaries.markers = []
        self._init_boundaries(self._num_markers_per_side,
                              self._boundary_width,
                              self._arena_width,
                              self._arena_height)

        # Setup Brick Publisher
        self._brick_publisher = self.create_publisher(Marker, '/brick',
                                                      markerQoS)
        self._brick_pose = PoseStamped()
        self._brick_marker = self._generate_marker(
            4*self._num_markers_per_side,
            self._brick_pose.pose.position.x,
            self._brick_pose.pose.position.y,
            self._brick_pose.pose.position.z,
            self._brick_pose.pose.orientation.x,
            self._brick_pose.pose.orientation.y,
            self._brick_pose.pose.orientation.z,
            self._brick_pose.pose.orientation.w,
            self._brick_size_x,
            self._brick_size_y,
            self._brick_size_z,
            r=1.0, g=0.0, b=0.0, a=1.0)

        # Setup Arena State Publisher
        self._arena_state_publisher = self.create_publisher(Float32,
                                                            '/arena_state',
                                                            10)
        self._arena_state_msg = Float32()

        # Setup subscriber for turtle's pose state
        self._turtle_pose_subscriber = self.create_subscription(
            Pose,
            self._robot_name+'/pose',
            self._turtle_pose_callback, 10)
        self._turtle_pose_subscriber  # prevent unused variable warning
        self._turtle_pose = Pose()

        # Setup subscriber for tilt's angle
        self._tilt_subscriber = self.create_subscription(
            Tilt,
            '/tilt',
            self._tilt_callback,
            10)
        self._tilt_subscriber  # prevent unused variable warning
        self._tilt_angle = 0.0

        # Setup /place Service
        self._place_service = self.create_service(Place,
                                                  '/place',
                                                  self._place_callback)

        # Setup /drop Service
        self._drop_service = self.create_service(Empty,
                                                 '/drop',
                                                 self._drop_callback)

        # Setup TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize arena state
        self._state = INIT

        # Initialize physics
        self._physics = World([self._brick_pose.pose.position.x,
                               self._brick_pose.pose.position.y,
                               self._brick_pose.pose.position.z],
                              self._gravity, self._platform_cylinder_radius,
                              1.0/self._physics_frequency)
        self._physics_z_limit = 0.0
        self._physics_pitch = 0.0

        # Initialize offsets for carrying the brick
        self._x_offset = 0.0
        self._y_offset = 0.0

    def _tilt_callback(self, msg):
        """Cache tilt angle updates.

        Args:
            msg (Tilt): The tilt angle message.
        """
        self._tilt_angle = msg.angle

    def _turtle_pose_callback(self, msg):
        """Receiving the turtle's pose.

        Args:
            msg (Pose): The current pose of the turtle in the arena.
        """
        self._turtle_pose.x = msg.x
        self._turtle_pose.y = msg.y
        self._turtle_pose.theta = msg.theta

    def _drop_callback(self, request, response):
        """Handle the '/drop' service, updating the arena state.

        Args:
            request (Empty): Empty service request.
            response (Empty): Empty service response.

        Returns:
            Empty: The response after handling the drop request.
        """
        if (self._state == PLACED):
            self._state = DROPPING
        return response

    def _place_callback(self, request, response):
        """Handle the '/place' service, updating brick position.

        Args:
            request (Place): Contains target position coordinates (x, y, z)
                             for the brick.
            response (Place.Response): Response indicating success or failure
                                       of placement.

        Returns:
            Place.Response: The updated response with success set to True.
        """
        # Update brick position
        new_position = [request.x, request.y, request.z]
        self._physics.brick = new_position
        response.success = True
        self._state = PLACED

        return response

    def _init_boundaries(self,
                         num_markers_per_side,
                         boundary_width,
                         arena_width,
                         arena_height):
        """Initialize arena boundaries with markers.

        Args:
            num_markers_per_side (int): Number of markers on each boundary
                                        side.
            boundary_width (float): Width of each boundary.
            arena_width (float): Width of the arena.
            arena_height (float): Height of the arena.
        """
        # Side 1
        for i in range(0, num_markers_per_side):
            x = -0.5-boundary_width/2
            y = (arena_width+boundary_width * 2) / num_markers_per_side *\
                (i + 0.5) - 0.5 - boundary_width
            z = arena_height/2
            R = 0
            P = 0
            Y = np.pi/2
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            scale_x = (arena_width+boundary_width*2)/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3],
                                           scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)

        # Side 2
        for i in range(num_markers_per_side, 2*num_markers_per_side):
            x = arena_width / num_markers_per_side *\
                (i-num_markers_per_side + 0.5) - 0.5
            y = arena_width-0.5+boundary_width/2
            z = arena_height/2
            R = 0
            P = 0
            Y = 0
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            scale_x = arena_width/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3],
                                           scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)

        # Side 3
        for i in range(2*num_markers_per_side, 3*num_markers_per_side):
            x = arena_width-0.5+boundary_width/2
            y = (arena_width + boundary_width * 2) / num_markers_per_side *\
                (3 * num_markers_per_side - i - 0.5) - 0.5-boundary_width
            z = arena_height/2
            R = 0
            P = 0
            Y = np.pi/2
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            scale_x = (arena_width+boundary_width*2)/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3],
                                           scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)

        # Side 4
        for i in range(3*num_markers_per_side, 4*num_markers_per_side):
            x = arena_width / num_markers_per_side *\
                (4 * num_markers_per_side - i - 0.5) - 0.5
            y = -0.5-boundary_width/2
            z = arena_height/2
            R = 0
            P = 0
            Y = 0
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            scale_x = arena_width/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3],
                                           scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)

    def _generate_marker(self, marker_id, x, y, z, qx, qy, qz, qw,
                         scale_x, scale_y, scale_z,
                         r=0.0, g=0.0, b=1.0, a=1.0):
        """Generate a visual marker with specified properties.

        Args:
            id (int): Unique ID for the marker.
            x, y, z (float): Position coordinates for the marker.
            qx, qy, qz, qw (float): Orientation quaternion values.
            scale_x, scale_y, scale_z (float): Scale dimensions of the marker.
            r, g, b, a (float): Color RGBA values.

        Returns:
            Marker: A configured marker object.
        """
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'boundary'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    def _main_loop_timer_callback(self):
        """Timer main loop, publishing state and boundary updates.

        Publishes the current state, updates boundaries, and handles brick
        pose based on the arena state and physics.
        """
        # Publish arena state
        self._arena_state_msg.data = float(self._state)
        self._arena_state_publisher.publish(self._arena_state_msg)

        # Publish boundaries
        self._boundaries_publisher.publish(self._boundaries)

        # Update brick pose based on physics
        self._brick_pose.pose.position.x = self._physics.brick[0]
        self._brick_pose.pose.position.y = self._physics.brick[1]
        self._brick_pose.pose.position.z = self._physics.brick[2]

        if (self._state != INIT):
            # Publish brick
            self._brick_marker.pose.position.x\
                = self._brick_pose.pose.position.x
            self._brick_marker.pose.position.y\
                = self._brick_pose.pose.position.y
            self._brick_marker.pose.position.z\
                = self._brick_pose.pose.position.z
            self._brick_publisher.publish(self._brick_marker)

            # Broadcast TF
            t = tf2_ros.TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'brick'
            t.transform.translation.x = self._brick_pose.pose.position.x
            t.transform.translation.y = self._brick_pose.pose.position.y
            t.transform.translation.z = self._brick_pose.pose.position.z
            t.transform.rotation.x = self._brick_pose.pose.orientation.x
            t.transform.rotation.y = self._brick_pose.pose.orientation.y
            t.transform.rotation.z = self._brick_pose.pose.orientation.z
            t.transform.rotation.w = self._brick_pose.pose.orientation.w
            self._tf_broadcaster.sendTransform(t)

        if (self._state == FALLING):
            # Brick's orientation is enabled
            R = 0.0
            P = self._tilt_angle
            Y = 0.0
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            self._brick_pose.pose.orientation.x = q[0]
            self._brick_pose.pose.orientation.y = q[1]
            self._brick_pose.pose.orientation.z = q[2]
            self._brick_pose.pose.orientation.w = q[3]
            self._brick_marker.pose.orientation.x = q[0]
            self._brick_marker.pose.orientation.y = q[1]
            self._brick_marker.pose.orientation.z = q[2]
            self._brick_marker.pose.orientation.w = q[3]

            # Check if brick fell off the platform
            dx = abs(self._brick_pose.pose.position.x - self._turtle_pose.x)
            if (self._physics._brick[2] > self._brick_size_z/2):
                # Brick is still on the platform
                self._physics_z_limit = self._platform_height +\
                    self._brick_size_z/2 - dx*np.tan(self._tilt_angle)
                self._physics_pitch = self._tilt_angle
                self._physics._brick = self._physics.drop(
                    z_limit=self._physics_z_limit, pitch=self._physics_pitch)
            else:
                # Brick fell off the platform
                self._physics_z_limit = 0.0
                self._physics_pitch = 0.0
                self.get_logger().info('Brick fell off the platform')
                self._state = SLIDING

        elif (self._state == SLIDING):
            # Check if brick is inside the arena
            self._physics._brick = self._physics.drop(
                z_limit=self._physics_z_limit, pitch=self._physics_pitch,
                friction_dx=self._friction_dx)
            x = self._physics._brick[0]
            y = self._physics._brick[1]
            if (self._physics._brick[0] <= -0.5+self._brick_size_x/2):
                # Brick is outside the arena
                x = -0.5+self._brick_size_x/2
                # Reverse x velocity to simulate collision
                self._physics._xdot = -self._physics._xdot
            if (self._physics._brick[0] >=
               (self._arena_width-0.5-self._brick_size_x/2)):
                # Brick is outside the arena
                x = self._arena_width-0.5-self._brick_size_x/2
                # Reverse x velocity to simulate collision
                self._physics._xdot = -self._physics._xdot
            if (self._physics._brick[1] <= -0.5+self._brick_size_y/2):
                # Brick is outside the arena
                y = -0.5+self._brick_size_y/2
            if (self._physics._brick[1] >=
               (self._arena_width-0.5-self._brick_size_y/2)):
                # Brick is outside the arena
                y = self._arena_width-0.5-self._brick_size_y/2
            if (self._physics._brick[2] == self._physics_z_limit):
                # Brick is on the ground, reset orientation
                self._brick_pose.pose.orientation.x = 0.0
                self._brick_pose.pose.orientation.y = 0.0
                self._brick_pose.pose.orientation.z = 0.0
                self._brick_pose.pose.orientation.w = 1.0
                self._brick_marker.pose.orientation.x = 0.0
                self._brick_marker.pose.orientation.y = 0.0
                self._brick_marker.pose.orientation.z = 0.0
                self._brick_marker.pose.orientation.w = 1.0
            # Update brick pose
            self._physics._brick[0] = x
            self._physics._brick[1] = y

        if (self._tilt_angle != 0.0 and self._state == DROPPED):
            # Brick is sliding
            self._state = FALLING

    def _physics_timer_callback(self):
        """Physics updates, handling brick dropping and sliding.

        Updates brick position based on physics and changes state as necessary
        based on the brick's position and interactions.
        """
        if (self._state == DROPPING):
            # Check if brick is inside the platform
            distance = np.sqrt((self._brick_pose.pose.position.x -
                                self._turtle_pose.x)**2 +
                               (self._brick_pose.pose.position.y -
                                self._turtle_pose.y)**2)
            if distance < self._tolerance:
                # Brick is inside the platform
                self._physics_z_limit = self._platform_height +\
                    self._brick_size_z/2
            else:
                # Brick is outside the platform
                self._physics_z_limit = 0.0

            self._physics._brick = self._physics.drop(
                z_limit=self._physics_z_limit)
            if (abs(self._physics._brick[2] - self._physics_z_limit) <
               self._tolerance):
                # Brick is on the platform
                self._x_offset = self._physics.brick[0] - self._turtle_pose.x
                self._y_offset = self._physics.brick[1] - self._turtle_pose.y
                self._state = DROPPED

        elif (self._state == DROPPED):
            # Move brick with the turtle
            self._physics.brick = [self._turtle_pose.x + self._x_offset,
                                   self._turtle_pose.y + self._y_offset,
                                   self._physics_z_limit]

        elif (self._state == FALLING):
            # Update brick position
            dx = self._brick_pose.pose.position.x - self._turtle_pose.x
            self._physics_z_limit = self._platform_height +\
                self._brick_size_z/2 - dx*np.tan(self._tilt_angle)
            self._physics_pitch = self._tilt_angle
            self._physics._brick = self._physics.drop(
                z_limit=self._physics_z_limit, pitch=self._physics_pitch)

    def _rainbow_timer_callback(self):
        """Dynamically changing marker colors based on a rainbow pattern.

        Uses time-based color adjustments to simulate a rainbow effect on
        boundary markers.
        """
        elapsed_time = time.time() - self._start_time
        for idx in range(self._num_markers_in_boundaries):
            color = self.get_rainbow_color(
                idx / self._num_markers_in_boundaries + elapsed_time)
            self._boundaries.markers[idx].color = color
        color = self.get_rainbow_color(
            1 / self._num_markers_in_boundaries + elapsed_time)
        self._brick_marker.color = color

    def get_rainbow_color(self, position):
        """Calculate color in the rainbow spectrum based on position.

        Args:
            position (float): Position value to generate the color

        Returns:
            ColorRGBA: Color with calculated RGB values and full opacity.
        """
        r = max(0.0, math.sin(position * 2 * math.pi + 0) * 0.5 + 0.5)
        g = max(0.0, math.sin(
            position * 2 * math.pi + 2 * math.pi / 3) * 0.5 + 0.5)
        b = max(0.0, math.sin(
            position * 2 * math.pi + 4 * math.pi / 3) * 0.5 + 0.5)
        return ColorRGBA(r=r, g=g, b=b, a=1.0)


def main(args=None):
    """Entry point for the ArenaNode.

    Initializes the ROS 2 node, spins to process callbacks, and shuts down.

    Args:
        args (list, optional): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    dummy_node = ArenaNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
