import unittest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing.markers
from launch_testing.actions import ReadyToTest
import rclpy
from geometry_msgs.msg import Twist
from launch_testing_ros import WaitForTopics

        
@pytest.mark.rostest
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Define and launch the turtle_robot node
    turtle_robot_node = LaunchNode(
        package='turtle_brick',
        executable='turtle_robot',
        name='turtle_robot'
    )
    
    return (
        LaunchDescription([
            turtle_robot_node,
            ReadyToTest()
        ]),
        {'myaction': turtle_robot_node}
    )


class TestTurtleRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs once when the test case is loaded"""
        rclpy.init()


    @classmethod
    def tearDownClass(cls):
        """Runs once when the test case is unloaded"""
        rclpy.shutdown()


    def setUp(self):
        self.dummy = True
        """Runs before every test"""
        self.node = rclpy.create_node('test_turtle_robot_node')
        self.cmd_vel_count = 0
        self.last_time = None
        self.received_rates = []
        self.ref_rate = 100.0
        self.minimum_messages = 90
        self.tolerance = 5

        # Subscriber for cmd_vel to measure the publishing rate
        self.subscription = self.node.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.cmd_vel_callback,
            10  # Queue size
        )


    def tearDown(self):
        """Runs after every test"""
        self.node.destroy_node()


    def cmd_vel_callback(self, msg):
        if(self.last_time == None):
            self.last_time = self.node.get_clock().now().nanoseconds / 1e9
        else:
            current_time = self.node.get_clock().now().nanoseconds / 1e9
            if self.last_time:
                elapsed_time = current_time - self.last_time
                rate = 1 / elapsed_time
                self.received_rates.append(rate)
            self.last_time = current_time
            self.cmd_vel_count += 1


    def test_publish_rate_100_hz(self):
        self.assertEqual(self.dummy, True,
                         'wrong size after resize')
        # Wait for the cmd_vel topic to be active
        wait_for_topics = WaitForTopics([('turtle1/cmd_vel', Twist)], timeout=5.0)
        self.assertEqual(wait_for_topics.wait(), True, "cmd_vel topic not available")

        # Run the test node for 5 second to accumulate data
        secs = 2.0
        start_time = self.node.get_clock().now().nanoseconds / 1e9
        timeout = start_time + secs  # Run for 5 second
        while rclpy.ok() and self.node.get_clock().now().nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)  # Small timeout for non-blocking

        # Verify if the publish rate is within the expected range
        self.assertEqual(len(self.received_rates)>self.minimum_messages, True, "Insufficient messages received for 100 Hz rate")
        average_rate = sum(self.received_rates) / len(self.received_rates)
        self.assertEqual(average_rate<self.ref_rate+self.tolerance, True, "Publishing rate out of expected range")
        self.assertEqual(average_rate>self.ref_rate-self.tolerance, True, "Publishing rate out of expected range")
        