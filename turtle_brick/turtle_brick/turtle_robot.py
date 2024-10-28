import rclpy
import tf_transformations
import tf2_ros
import numpy as np

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from turtle_brick_interfaces.msg import Tilt
from std_srvs.srv import Empty

from turtle_brick.states import MOVING, STOPPED, REACHED
from turtle_brick.holonomic_controller import HolonomicController


class TurtleRobotNode(Node):
    def __init__(self):
        super().__init__('turtle_robot')
        # Declare ROS 2 parameters
        # Declare timer frequency parameter, default to 100 Hz
        self.declare_parameter('frequency', 100.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare goal max_velocity parameter, default to 3.0
        self.declare_parameter('max_velocity', 3.0)
        self._max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        # Declare goal kp parameter, default to 0.1
        self.declare_parameter('kp', 0.1)
        self._kp = self.get_parameter("kp").get_parameter_value().double_value
        # Declare robot urdf parameter, default to 0.1
        self.declare_parameter('base_link_length', 0.1)
        self._base_link_length = self.get_parameter("base_link_length").get_parameter_value().double_value
        self.declare_parameter('stem_height', 0.1)
        self._stem_height = self.get_parameter("stem_height").get_parameter_value().double_value
        self.declare_parameter('wheel_radius', 0.1)
        self._wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self._base_to_footprint_z = self._stem_height + self._wheel_radius*2 + self._base_link_length/2
        self.declare_parameter('platform_cylinder_radius', 0.1)
        self._platform_cylinder_radius = self.get_parameter("platform_cylinder_radius").get_parameter_value().double_value
        self._tolerance = self._platform_cylinder_radius/2
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
            
        # Setup main loop Timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup subscriber for turtle's pose state
        self._turtle_pose_subscriber = self.create_subscription(Pose, self._robot_name+'/pose', self._turtle_pose_callback, 10)
        self._turtle_pose_subscriber  # prevent unused variable warning
        
        # Setup subscriber for turtle's goal pose
        self._goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_callback, 10)
        self._goal_pose_subscriber  # prevent unused variable warning
        
        # Setup subscriber for tilt's angle
        self._tilt_subscriber = self.create_subscription(Tilt, '/tilt', self._tilt_callback, 10)
        self._tilt_subscriber  # prevent unused variable warning
        self._tilt_angle = 0
        
        # Setup publisher to control the turtle's movements
        self._turtle_cmd_publisher = self.create_publisher(Twist, self._robot_name+'/cmd_vel', 10)
        self._turtle_cmd = Twist()
        self._vx = 0
        self._vy = 0
        self._th = 0
        
        # Setup publisher to publish joint states
        self._joint_states_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._joint_states = JointState()
        self._joint_states.name = ['platform_connector_to_platform', 
                                   'base_link_to_stem',
                                   'stem_to_wheel']
        
        # Setup TF broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize turtle pose
        self._turtle_pose = Pose()
        self._goal_pose = PoseStamped()
        
        # Initialize state
        self._state = STOPPED

        # Initialize motion controller
        self._controller = HolonomicController(self._max_velocity, self._kp)
        
        # Initialize wheel position
        self._wheel_position = 0
        
        
    def _timer_callback(self):
        # Publish turtle pose to TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._turtle_pose.x
        t.transform.translation.y = self._turtle_pose.y
        t.transform.translation.z = self._base_to_footprint_z
        q = tf_transformations.quaternion_from_euler(0, 0, self._turtle_pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(t)
        
        
        # Check if goal is reached
        if(self._state == MOVING):
            p1 = [self._turtle_pose.x, self._turtle_pose.y]
            p2 = [self._goal_pose.pose.position.x, self._goal_pose.pose.position.y]
            # if(self._euclidean_distance(p1, p2) < self._tolerance):
            if(self._euclidean_distance(p1, p2) < self._tolerance):
                self._vx, self._vy = 0.0, 0.0
                self._turtle_cmd = Twist()
                self._turtle_cmd_publisher.publish(self._turtle_cmd)
                self._state = STOPPED
            else:
                # Use planner to calc control commands
                current_state = [self._turtle_pose.x, self._turtle_pose.y, self._turtle_pose.theta]
                goal_pose = [self._goal_pose.pose.position.x, self._goal_pose.pose.position.y]
                self._vx, self._vy, self._th = self._controller.holonomic_control(current_state, goal_pose)
                self._turtle_cmd.linear.x = self._vx
                self._turtle_cmd.linear.y = self._vy
                self._turtle_cmd_publisher.publish(self._turtle_cmd)
            
        elif(self._state == STOPPED):
            self._turtle_cmd = Twist()
            
        else:
            self.get_logger().warn("Unknown state: "+str(self._state))
        
        self._turtle_cmd_publisher.publish(self._turtle_cmd)
            
        
        
        # Publish joint states
        self._wheel_position += abs(self._vx) * 1.0/self._timer_frequency
        self._joint_states.header.stamp = self.get_clock().now().to_msg()
        self._joint_states.position = [self._tilt_angle, self._th, self._wheel_position]
        self._joint_states.velocity = [0, 0, 0]
        self._joint_states.effort = [0, 0, 0]
        self._joint_states_publisher.publish(self._joint_states)
    
    
    def _goal_pose_callback(self, msg):
        if(self._state == STOPPED):
            self._state = MOVING
        self._goal_pose = msg
        
        
    def _turtle_pose_callback(self, msg):
        self._turtle_pose.x = msg.x
        self._turtle_pose.y = msg.y
    
    
    def _tilt_callback(self, msg):
        self._tilt_angle = msg.angle
        
        
    def _euclidean_distance(self, p1, p2):
        return np.sqrt(((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))
        
        
def main(args=None):
    rclpy.init(args=args)
    turtle_robot_node = TurtleRobotNode()
    rclpy.spin(turtle_robot_node)
    turtle_robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()