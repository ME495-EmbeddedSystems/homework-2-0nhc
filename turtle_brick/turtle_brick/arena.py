import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
import tf_transformations
import numpy as np
import tf2_ros
from turtle_brick.states import INIT, PLACED, DROPPING
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import Place
from std_srvs.srv import Empty 


class ArenaNode(Node):

    def __init__(self):
        super().__init__('arena')
        
        # Setup Main Loop Timer
        frequency = 250.0
        self._timer = self.create_timer(1.0/frequency, self._timer_callback)
        
        # Setup Boundaries Publisher
        self._boundaries_publisher = self.create_publisher(MarkerArray, '/boundaries', 10)
        self._boundaries = MarkerArray()
        self._boundaries.markers = []
        num_markers_per_side = 50
        boundary_width = float(0.3)
        arena_width = 12
        arena_height = 0.8
        self._init_boundaries(num_markers_per_side, boundary_width, arena_width, arena_height)
        
        # Setup Brick Publisher
        self._brick_publisher = self.create_publisher(Marker, '/brick', 10)
        brick_size_x = 0.2
        brick_size_y = 0.15
        brick_size_z = 0.075
        self._brick_pose = PoseStamped()
        self._brick_marker = self._generate_marker(4*num_markers_per_side, 
                                                   self._brick_pose.pose.position.x, 
                                                   self._brick_pose.pose.position.y, 
                                                   self._brick_pose.pose.position.z, 
                                                   self._brick_pose.pose.orientation.x,
                                                   self._brick_pose.pose.orientation.y,
                                                   self._brick_pose.pose.orientation.z,
                                                   self._brick_pose.pose.orientation.w,
                                                   brick_size_x, 
                                                   brick_size_y, 
                                                   brick_size_z,
                                                   r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Setup /place Service
        self._place_service = self.create_service(Place, '/place', self._place_callback)
        
        # Setup /drop Service
        self._drop_service = self.create_service(Empty, '/drop', self._drop_callback)
        
        # Setup TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize arena state
        self._state = INIT
        
        # Initialize physics
        radius = 0.3
        gravity = -9.81
        self._physics = World([self._brick_pose.pose.position.x,
                             self._brick_pose.pose.position.y,
                             self._brick_pose.pose.position.z], 
                            gravity, radius, 1.0/frequency)
    
    
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
                self._physics._brick = self._physics.drop()


def main(args=None):
    rclpy.init(args=args)
    dummy_node = ArenaNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()