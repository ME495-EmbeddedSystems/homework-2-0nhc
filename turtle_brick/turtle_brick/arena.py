import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import tf_transformations
import numpy as np


class ArenaNode(Node):

    def __init__(self):
        super().__init__('arena')
        
        # Setup Main Loop Timer
        self._timer = self.create_timer(0.01, self._timer_callback)
        
        # Setup Markers Publisher
        self._markers_publisher = self.create_publisher(MarkerArray, '/boundaries', 10)
        self._boundaries = MarkerArray()
        self._boundaries.markers = []
        
        # number of markers in one side of the arena
        num_markers_per_side = 50
        boundary_width = float(0.3)
        arena_width = 12
        arena_height = 0.8
        self._init_markers(num_markers_per_side, boundary_width, arena_width, arena_height)
    
    def _init_markers(self, num_markers_per_side, boundary_width, arena_width, arena_height):
        # side 1
        for i in range(0, num_markers_per_side):
            x=-0.5-boundary_width/2
            y=(arena_width+boundary_width*2)/num_markers_per_side*(i+0.5)-0.5-boundary_width
            z=arena_height/2
            R = 0
            P = 0
            Y = np.pi/2
            scale_x = (arena_width+boundary_width*2)/num_markers_per_side
            scale_y = boundary_width
            scale_z = arena_height
            marker = self._generate_marker(i, x, y, z, R, P, Y, scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
        # side 2
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
            marker = self._generate_marker(i, x, y, z, R, P, Y, scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
        # side 3
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
            marker = self._generate_marker(i, x, y, z, R, P, Y, scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
        # side 4
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
            marker = self._generate_marker(i, x, y, z, R, P, Y, scale_x, scale_y, scale_z)
            self._boundaries.markers.append(marker)
        
        
    def _generate_marker(self, id, x, y, z, R, P, Y, scale_x, scale_y, scale_z):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "boundary"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        q = tf_transformations.quaternion_from_euler(R, P, Y)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker
        
        
    def _timer_callback(self):
        self._markers_publisher.publish(self._boundaries)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = ArenaNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()