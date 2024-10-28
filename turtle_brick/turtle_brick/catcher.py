import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtle_brick_interfaces.msg import Tilt
from std_msgs.msg import Float32
from turtle_brick.states import INIT, PLACED, DROPPING, DROPPED, FALLING, CATCHING, NONAVAILABLE, CATCHED, CLEARING
from turtlesim.msg import Pose
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class CatcherNode(Node):

    def __init__(self):
        super().__init__('catcher')
        # Declare timer frequency parameter, default to 100 Hz
        self.declare_parameter('frequency', 100.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare robot urdf parameter, default to 0.1
        self.declare_parameter('platform_height', 0.1)
        self._platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        # Declare brick size z parameter, default to 0.075
        self.declare_parameter('brick_size_z', 0.075)
        self._brick_size_z = self.get_parameter("brick_size_z").get_parameter_value().double_value
        # Declare goal max_velocity parameter, default to 3.0
        self.declare_parameter('max_velocity', 3.0)
        self._max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        # Declare gravity acceleration parameter, default to -9.81
        self.declare_parameter('gravity', -9.81)
        self._gravity = self.get_parameter("gravity").get_parameter_value().double_value
        # Declare number of markers per side parameter, default to 20
        self.declare_parameter('num_markers_per_side', 50)
        self._num_markers_per_side = self.get_parameter("num_markers_per_side").get_parameter_value().integer_value
        # Declare goal tolerance parameter, default to 0.1
        self.declare_parameter('tolerance', 0.1)
        self._tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
        
        # Setup Main Loop Timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup Tilt Publisher
        self._tilt_publisher = self.create_publisher(Tilt, '/tilt', 10)
        self._tilt_msg = Tilt()
        
        # Setup goal_pose Publisher
        self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._goal_pose = PoseStamped()
        
        # Setup words marker Publisher
        self._words_publisher = self.create_publisher(Marker, '/words', 10)
        self._words = Marker()
        self._words.header.frame_id = 'odom'
        self._words.ns = 'catcher'
        self._words.id = 4*self._num_markers_per_side+1
        self._words.type = Marker.TEXT_VIEW_FACING
        self._words.action = Marker.ADD
        self._words.pose.position.x = 5.544445
        self._words.pose.position.y = 5.544445
        self._words.pose.position.z = 1.5
        self._words.pose.orientation.x = 0.0
        self._words.pose.orientation.y = 0.0
        self._words.pose.orientation.z = 0.0
        self._words.pose.orientation.w = 1.0
        self._words.text = "Unreachable"
        self._words.scale.z = 1.5  # Height of the text in meters
        self._words.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color, fully opaque
        self._words.lifetime.sec = 3
        self._words.lifetime.nanosec = 0
        
        
        
        # Setup subscriber for arena's state
        self._arena_state_subscriber = self.create_subscription(Float32, '/arena_state', self._arena_state_callback, 10)
        self._arena_state_subscriber  # prevent unused variable warning
        self._arena_state = INIT
        
        # Setup subscriber for turtle's pose state
        self._turtle_pose_subscriber = self.create_subscription(Pose, self._robot_name+'/pose', self._turtle_pose_callback, 10)
        self._turtle_pose = Pose()
        
        # Setup subscriber for brick's pose state
        self._brick_subscriber = self.create_subscription(Marker, '/brick', self._brick_callback, 10)
        self._brick_pose = Pose()
        
        # Initialize catcher state
        self._state = INIT
        

    def _turtle_pose_callback(self, msg):
        self._turtle_pose.x = msg.x
        self._turtle_pose.y = msg.y
        self._turtle_pose.theta = msg.theta
        
        
    def _brick_callback(self, msg):
        self._brick_pose = msg.pose
        
        
    def _arena_state_callback(self, msg):
        self._arena_state = msg.data
        
        
    def _timer_callback(self):
        if(self._arena_state == DROPPING and self._state == INIT):
            # Check if the robot can move to catch the brick on time
            rx, ry = self._turtle_pose.x, self._turtle_pose.y
            bx, by, bz = self._brick_pose.position.x, self._brick_pose.position.y, self._brick_pose.position.z
            
            distance = ((rx-bx)**2 + (ry-by)**2)**0.5
            time_to_catch = distance/self._max_velocity
            
            z = bz - self._brick_size_z/2 - self._platform_height
            time_to_drop = np.sqrt(-2.0*z/self._gravity)
            
            if(time_to_catch <= time_to_drop):
                self._state = CATCHING
            else:
                self._state = NONAVAILABLE
                
        
        if(self._state == CATCHING):
            # Turn the tilt to zero
            self._tilt_msg.angle = 0.0
            self._tilt_publisher.publish(self._tilt_msg)
            
            # Move the robot to catch the brick
            self._goal_pose.pose.position.x = self._brick_pose.position.x
            self._goal_pose.pose.position.y = self._brick_pose.position.y
            self._goal_pose_publisher.publish(self._goal_pose)
            
            if(self._arena_state == DROPPED):
                self._state = CATCHED

        
        elif(self._state == CATCHED):
            # Move the robot to the center of the arena
            self._goal_pose.pose.position.x = 5.544445
            self._goal_pose.pose.position.y = 5.544445
            self._goal_pose_publisher.publish(self._goal_pose)
            
            # Check if the robot has reached the center of the arena
            distance = ((self._turtle_pose.x-5.544445)**2 + (self._turtle_pose.y-5.544445)**2)**0.5
            if(distance < self._tolerance):
                self._state = CLEARING
        
        elif(self._state == CLEARING):
            # Turn the platform to clear the brick
            self._tilt_msg.angle = 0.35
            self._tilt_publisher.publish(self._tilt_msg)
            
        elif(self._state == NONAVAILABLE):
            self._words_publisher.publish(self._words)
            self._state = INIT
            
        # self._tilt_publisher.publish(self._tilt_msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = CatcherNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    