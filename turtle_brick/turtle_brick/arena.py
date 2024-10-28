import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
import tf_transformations
import numpy as np
import tf2_ros
from turtle_brick.states import INIT, PLACED, DROPPING, DROPPED, FALLING, SLIDING
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import Place
from turtle_brick_interfaces.msg import Tilt
from std_srvs.srv import Empty 
from turtlesim.msg import Pose


class ArenaNode(Node):

    def __init__(self):
        super().__init__('arena')
        # Declare timer frequency parameter, default to 100 Hz
        self.declare_parameter('frequency', 100.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare physics frequency parameter, default to 250 Hz
        self.declare_parameter('physics_frequency', 250.0)
        self._physics_frequency = self.get_parameter("physics_frequency").get_parameter_value().double_value
        # Declare number of markers per side parameter, default to 20
        self.declare_parameter('num_markers_per_side', 50)
        self._num_markers_per_side = self.get_parameter("num_markers_per_side").get_parameter_value().integer_value
        # Declare boundary width parameter, default to 0.3
        self.declare_parameter('boundary_width', 0.3)
        self._boundary_width = self.get_parameter("boundary_width").get_parameter_value().double_value
        # Declare arena width parameter, default to 12.0
        self.declare_parameter('arena_width', 12.0)
        self._arena_width = self.get_parameter("arena_width").get_parameter_value().double_value
        # Declare arena height parameter, default to 0.8
        self.declare_parameter('arena_height', 0.8)
        self._arena_height = self.get_parameter("arena_height").get_parameter_value().double_value
        # Declare brick size x parameter, default to 0.2
        self.declare_parameter('brick_size_x', 0.2)
        self._brick_size_x = self.get_parameter("brick_size_x").get_parameter_value().double_value
        # Declare brick size y parameter, default to 0.15
        self.declare_parameter('brick_size_y', 0.15)
        self._brick_size_y = self.get_parameter("brick_size_y").get_parameter_value().double_value
        # Declare brick size z parameter, default to 0.075
        self.declare_parameter('brick_size_z', 0.075)
        self._brick_size_z = self.get_parameter("brick_size_z").get_parameter_value().double_value
        # Declare gravity acceleration parameter, default to -9.81
        self.declare_parameter('gravity', -9.81)
        self._gravity = self.get_parameter("gravity").get_parameter_value().double_value
        # Declare robot urdf parameter, default to 0.1
        self.declare_parameter('platform_height', 0.1)
        self._platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.declare_parameter('platform_cylinder_radius', 0.1)
        self._platform_cylinder_radius = self.get_parameter("platform_cylinder_radius").get_parameter_value().double_value
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
        
        # Setup Main Loop Timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup Boundaries Publisher
        self._boundaries_publisher = self.create_publisher(MarkerArray, '/boundaries', 10)
        self._boundaries = MarkerArray()
        self._boundaries.markers = []
        self._init_boundaries(self._num_markers_per_side, self._boundary_width, self._arena_width, self._arena_height)
        
        # Setup Brick Publisher
        self._brick_publisher = self.create_publisher(Marker, '/brick', 10)
        self._brick_pose = PoseStamped()
        self._brick_marker = self._generate_marker(4*self._num_markers_per_side, 
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
        
        # Setup subscriber for turtle's pose state
        self._turtle_pose_subscriber = self.create_subscription(Pose, self._robot_name+'/pose', self._turtle_pose_callback, 10)
        self._turtle_pose_subscriber  # prevent unused variable warning
        self._turtle_pose = Pose()
        
        # Setup subscriber for tilt's angle
        self._tilt_subscriber = self.create_subscription(Tilt, '/tilt', self._tilt_callback, 10)
        self._tilt_subscriber  # prevent unused variable warning
        self._tilt_angle = 0.0
        
        # Setup /place Service
        self._place_service = self.create_service(Place, '/place', self._place_callback)
        
        # Setup /drop Service
        self._drop_service = self.create_service(Empty, '/drop', self._drop_callback)
        
        # Setup TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize arena state
        self._state = INIT
        
        # Initialize physics
        self._physics = World([self._brick_pose.pose.position.x,
                             self._brick_pose.pose.position.y,
                             self._brick_pose.pose.position.z], 
                            self._gravity, self._platform_cylinder_radius, 1.0/self._physics_frequency)
        self._physics_z_limit = 0.0
        self._physics_pitch = 0.0
        
        # Initialize offsets for carrying the brick
        self._x_offset = 0.0
        self._y_offset = 0.0
    

    def _tilt_callback(self, msg):
        self._tilt_angle = msg.angle
        
            
    def _turtle_pose_callback(self, msg):
        self._turtle_pose.x = msg.x
        self._turtle_pose.y = msg.y
        self._turtle_pose.theta = msg.theta
        
        
    def _drop_callback(self, request, response):
        if(self._state == PLACED):
            self._state = DROPPING
        return response
    
    
    def _place_callback(self, request, response):
        new_position = [request.x, request.y, request.z]
        self._physics.brick = new_position
        response.success = True
        self._state = PLACED
        return response
    
    
    def _init_boundaries(self, num_markers_per_side, boundary_width, arena_width, arena_height):
        # Side 1
        for i in range(0, num_markers_per_side):
            x=-0.5-boundary_width/2
            y=(arena_width+boundary_width*2)/num_markers_per_side*(i+0.5)-0.5-boundary_width
            z=arena_height/2
            R = 0
            P = 0
            Y = np.pi/2
            q = tf_transformations.quaternion_from_euler(R, P, Y)
            scale_x = (arena_width+boundary_width*2)/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3], scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
            
        # Side 2
        for i in range(num_markers_per_side, 2*num_markers_per_side):
            x=arena_width/num_markers_per_side*(i-num_markers_per_side+0.5)-0.5
            y=arena_width-0.5+boundary_width/2
            z=arena_height/2
            R = 0
            P = 0
            Y = 0
            scale_x = arena_width/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3], scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
            
        # Side 3
        for i in range(2*num_markers_per_side, 3*num_markers_per_side):
            x=arena_width-0.5+boundary_width/2
            y=(arena_width+boundary_width*2)/num_markers_per_side*(-2*num_markers_per_side+i+0.5)-0.5-boundary_width
            z=arena_height/2
            R = 0
            P = 0
            Y = np.pi/2
            scale_x = (arena_width+boundary_width*2)/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3], scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
            
        # Side 4
        for i in range(3*num_markers_per_side, 4*num_markers_per_side):
            x=arena_width/num_markers_per_side*(-3*num_markers_per_side+i+0.5)-0.5
            y=-0.5-boundary_width/2
            z=arena_height/2
            R = 0
            P = 0
            Y = 0
            scale_x = arena_width/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, q[0], q[1], q[2], q[3], scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
        
        
    def _generate_marker(self, id, x, y, z, qx, qy, qz, qw, scale_x, scale_y, scale_z, r=0.0, g=0.0, b=1.0, a=1.0):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "boundary"
        marker.id = id
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
        
        
    def _timer_callback(self):
        # Publish boundaries
        self._boundaries_publisher.publish(self._boundaries)
        
        # Update brick pose based on physics
        self._brick_pose.pose.position.x = self._physics.brick[0]
        self._brick_pose.pose.position.y = self._physics.brick[1]
        self._brick_pose.pose.position.z = self._physics.brick[2]
        
        if(self._state != INIT):    
            # Publish brick
            self._brick_marker.pose.position.x = self._brick_pose.pose.position.x
            self._brick_marker.pose.position.y = self._brick_pose.pose.position.y
            self._brick_marker.pose.position.z = self._brick_pose.pose.position.z
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
            
        if(self._state == DROPPING):
            # Check if brick is inside the platform
            distance = np.sqrt((self._brick_pose.pose.position.x - self._turtle_pose.x)**2+
                                (self._brick_pose.pose.position.y - self._turtle_pose.y)**2)
            if distance < self._platform_cylinder_radius:
                # Brick is inside the platform
                self._physics_z_limit = self._platform_height + self._brick_size_z/2
            else:
                # Brick is outside the platform
                self._physics_z_limit = 0.0
                
            self._physics._brick = self._physics.drop(self._physics_z_limit)
            if(self._physics._brick[2] == self._physics_z_limit):
                self._x_offset = self._physics.brick[0] - self._turtle_pose.x
                self._y_offset = self._physics.brick[1] - self._turtle_pose.y
                self._state = DROPPED
        
        elif(self._state == DROPPED):
            # Move brick with the turtle
            self._physics.brick = [self._turtle_pose.x + self._x_offset, self._turtle_pose.y + self._y_offset, self._platform_height + self._brick_size_z/2]
        
        elif(self._state == FALLING):
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
            
            dx = self._brick_pose.pose.position.x - self._turtle_pose.x
            if(abs(dx) < self._platform_cylinder_radius):
                self._physics_z_limit = self._platform_height + self._brick_size_z/2 - dx*np.tan(self._tilt_angle)
                self._physics_pitch = self._tilt_angle
                self._physics._brick = self._physics.drop(z_limit=self._physics_z_limit, pitch=self._physics_pitch)
            else:
                self._physics_z_limit = 0.0
                self._physics_pitch = 0.0

                self._brick_pose.pose.orientation.x = 0.0
                self._brick_pose.pose.orientation.y = 0.0
                self._brick_pose.pose.orientation.z = 0.0
                self._brick_pose.pose.orientation.w = 1.0
                self._brick_marker.pose.orientation.x = 0.0
                self._brick_marker.pose.orientation.y = 0.0
                self._brick_marker.pose.orientation.z = 0.0
                self._brick_marker.pose.orientation.w = 1.0
                self.get_logger().info("Brick fell off the platform")
                self._state = SLIDING
        
        elif(self._state == SLIDING):
            self.get_logger().info(f"{self._physics_z_limit}")
            self._physics._brick = self._physics.drop(z_limit=self._physics_z_limit, pitch=self._physics_pitch)
            
            x = self._physics._brick[0]
            y = self._physics._brick[1]
            
            if(self._physics._brick[0] <= -0.5+self._brick_size_x/2):
                x = -0.5+self._brick_size_x/2
                self._physics._xdot = -self._physics._xdot
            if(self._physics._brick[0] >= self._arena_width-0.5-self._brick_size_x/2):
                x = self._arena_width-0.5-self._brick_size_x/2
                self._physics._xdot = -self._physics._xdot
            if(self._physics._brick[1] <= -0.5+self._brick_size_y/2):
                y = -0.5+self._brick_size_y/2
            if(self._physics._brick[1] >= self._arena_width-0.5-self._brick_size_y/2):
                y = self._arena_width-0.5-self._brick_size_y/2
                
            self._physics._brick[0] = x
            self._physics._brick[1] = y
            
        
        if(self._tilt_angle != 0.0 and self._state == DROPPED):
            self._state = FALLING
                

def main(args=None):
    rclpy.init(args=args)
    dummy_node = ArenaNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()