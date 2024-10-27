import rclpy
import tf_transformations
import tf2_ros

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from turtle_brick.states import MOVING, STOPPED
from turtle_brick.holonomic_odometry import HolonomicOdometry


class TurtleRobotNode(Node):
    def __init__(self):
        super().__init__('turtle_robot')
        # Declare ROS 2 parameters
        # Declare timer frequency parameter, default to 100 Hz
        self.declare_parameter('frequency', 100.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        
        # Declare goal tolerance parameter, default to 0.1
        self.declare_parameter('tolerance', 0.1)
        self._tolerance = self.get_parameter("tolerance").get_parameter_value().double_value

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
        
        # Setup subscriber for turtle's gpal pose
        self._goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_callback, 10)
        self._goal_pose_subscriber  # prevent unused variable warning
        
        # Setup publisher to control the turtle's movements
        self._turtle_cmd_publisher = self.create_publisher(Twist, self._robot_name+'/cmd_vel', 10)
        self._turtle_cmd = Twist()
        
        # Setup publisher to publish joint states
        self._joint_states_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._joint_states = JointState()
        
        # Setup TF broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize turtle pose
        self._turtle_pose = Pose()
        
        # Initialize state
        self._state = STOPPED
        
        
    def _timer_callback(self):
        # Publish turtle pose to TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._turtle_pose.x
        t.transform.translation.y = self._turtle_pose.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self._turtle_pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(t)
        
        # Publish joint states
        self._joint_states.header.stamp = self.get_clock().now().to_msg()
        self._joint_states.name = ['platform_connector_to_platform', 
                                   'base_link_to_stem',
                                   'stem_to_wheel']
        self._joint_states.position = [0, 0, 0]
        self._joint_states.velocity = [0, 0, 0]
        self._joint_states.effort = [0, 0, 0]
        self._joint_states_publisher.publish(self._joint_states)
        
        # Publish control commands
        if(self._state == MOVING):
            # Use planner to calc control commands
            pass
        
        elif(self._state == STOPPED):
            self._turtle_cmd = Twist()
            
        else:
            self.get_logger().warn("Unknown state: "+str(self._state))
            
        self._turtle_cmd_publisher.publish(self._turtle_cmd)
    
    
    def _goal_pose_callback(self, msg):
        self._goal_pose = msg
        
        
    def _turtle_pose_callback(self, msg):
        self._turtle_pose = msg
        
        
def main(args=None):
    rclpy.init(args=args)
    turtle_robot_node = TurtleRobotNode()
    rclpy.spin(turtle_robot_node)
    turtle_robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()