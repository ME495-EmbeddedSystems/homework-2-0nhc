import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DummyNode(Node):

    def __init__(self):
        super().__init__('dummy')
        
        # Setup ROS 2 Timer
        self._timer = self.create_timer(0.01, self._timer_callback)
        
        # Setup ROS 2 Publisher
        self._dummy_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def _timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        self._dummy_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()