import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtle_brick_interfaces.msg import Tilt
from std_msgs.msg import Float32
from turtle_brick.states import INIT, PLACED, DROPPING, DROPPED, FALLING

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
        
        # Setup Main Loop Timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup Tilt Publisher
        self._tilt_publisher = self.create_publisher(Tilt, '/tilt', 10)
        self._tilt_msg = Tilt()
        
        # Setup subscriber for arena's state
        self._arena_state_subscriber = self.create_subscription(Float32, '/arena_state', self._arena_state_callback, 10)
        self._arena_state_subscriber  # prevent unused variable warning
        self._arena_state = INIT
        
        
    def _arena_state_callback(self, msg):
        self._arena_state = msg.data
        
        
    def _timer_callback(self):
        if(self._arena_state == DROPPING):
            # Check if the robot can move to catch the brick on time
            pass
        
        # self._tilt_publisher.publish(self._tilt_msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = CatcherNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    