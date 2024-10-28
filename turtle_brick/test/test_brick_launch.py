import unittest
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
from launch_testing import WaitForTopics


@pytest.mark.rostest
def generate_test_description():
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


class TestTurtleRobotPublishRate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs once when the test case is loaded"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Runs once when the test case is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test"""
        self.node = rclpy.create_node('test_turtle_robot_node')
        self.cmd_vel_count = 0
        self.last_time = None
        self.received_rates = []

        # Subscriber for cmd_vel to measure the publishing rate
        self.subscription = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # Queue size
        )

    def tearDown(self):
        """Runs after every test"""
        self.node.destroy_node()

    def cmd_vel_callback(self, msg):
        """Callback to measure the publishing rate"""
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if self.last_time:
            elapsed_time = current_time - self.last_time
            rate = 1 / elapsed_time
            self.received_rates.append(rate)
        self.last_time = current_time
        self.cmd_vel_count += 1

    def test_publish_rate_100_hz(self, launch_service, myaction, proc_output):
        """Verify that the node publishes cmd_vel at 100 Hz"""

        # Wait for the cmd_vel topic to be active
        wait_for_topics = WaitForTopics([('cmd_vel', Twist)], timeout=5.0)
        assert wait_for_topics.wait(), "cmd_vel topic not available"

        # Run the test node for 1 second to accumulate data
        start_time = self.node.get_clock().now().nanoseconds / 1e9
        timeout = start_time + 1.0  # Run for 1 second
        while rclpy.ok() and self.node.get_clock().now().nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)  # Small timeout for non-blocking

        # Verify if the publish rate is within the expected range
        assert len(self.received_rates) > 90, "Insufficient messages received for 100 Hz rate"
        for rate in self.received_rates:
            assert 95 <= rate <= 105, f"Publishing rate out of expected range: {rate} Hz"
