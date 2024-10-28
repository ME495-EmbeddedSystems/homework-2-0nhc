import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtle_brick_interfaces.msg import Tilt


class DummyNode(Node):

    def __init__(self):
        super().__init__('debug_goal_pose')
        
        # Setup ROS 2 Timer
        self._timer = self.create_timer(0.01, self._timer_callback)
        
        # Setup ROS 2 Publisher
        self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def _timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = 7.0
        msg.pose.position.y = 2.0
        self._goal_pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()