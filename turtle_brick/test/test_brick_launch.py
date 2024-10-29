#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""
Testing file for turtle_robot's publish rate.

This module contains tests for the turtle_robot node in the turtle_brick
package.
"""
import unittest

from geometry_msgs.msg import Twist

from launch import LaunchDescription

from launch_ros.actions import Node as LaunchNode

from launch_testing.actions import ReadyToTest
import launch_testing.markers

from launch_testing_ros import WaitForTopics

import pytest

import rclpy


@pytest.mark.rostest
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Generate a launch description for testing the turtle_robot node.

    Launches the `turtle_robot` node from the `turtle_brick` package,
    marking it as ready for testing.

    Returns
    -------
        tuple: A tuple containing:
            - LaunchDescription: The launch description with the turtle_
              robot node and ReadyToTest action.
            - dict: Dictionary with a reference to the launched node.

    """
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
    """
    Test case for validating the turtle_robot node in the turtle_brick package.

    Test case for validating the publishing rate of the turtle_robot's
    cmd_vel topic.
    """

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS 2 system once before all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS 2 system once after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """
        Set up the test environment for each test.

        Creates a node to listen to the `turtle1/cmd_vel` topic, initializes
        tracking variables, and defines expected publish rate and tolerance
        for cmd_vel messages.
        """
        self.dummy = True
        self.node = rclpy.create_node('test_turtle_robot_node')
        self.cmd_vel_count = 0
        self.last_time = None
        self.received_rates = []
        self.ref_rate = 100.0
        self.minimum_messages = 100
        self.tolerance = 5.0
        self.secs = 30.0

        # Subscriber for cmd_vel to measure the publishing rate
        self.subscription = self.node.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.cmd_vel_callback,
            10  # Queue size
        )

    def tearDown(self):
        """Clean up after each test by destroying the test node."""
        self.node.destroy_node()

    def cmd_vel_callback(self, msg):
        """
        Record message receipt times and calculate publish rates.

        Measures the rate of incoming messages on the `turtle1/cmd_vel` topic
        and stores each calculated rate in `received_rates`.

        :param msg: The message received from the `turtle1/cmd_vel` topic.
        :type msg: geometry_msgs.msg.Twist
        """
        if (self.last_time is None):
            self.last_time = self.node.get_clock().now().nanoseconds / 1e9
        else:
            if (len(self.received_rates) >= self.minimum_messages):
                pass
            else:
                current_time = self.node.get_clock().now().nanoseconds / 1e9
                elapsed_time = current_time - self.last_time
                rate = 1.0 / elapsed_time
                self.received_rates.append(rate)
                self.last_time = current_time
                self.cmd_vel_count += 1

    def test_publish_rate_100_hz(self):
        """
        Test that the turtle_robot node publishes messages at 100 Hz.

        Waits for the `cmd_vel` topic to be active and accumulates message
        data for a set time period. Then, checks that the average publish
        rate falls within the specified tolerance of the target rate.

        Raises
        ------
            AssertionError: If the `cmd_vel` topic is not available, if
                            insufficient messages are received,
            or if the publish rate is outside the acceptable range.

        """
        self.assertEqual(self.dummy, True,
                         'wrong size after resize')
        # Wait for the cmd_vel topic to be active
        wait_for_topics = WaitForTopics(
            [('turtle1/cmd_vel', Twist)], timeout=15.0)
        self.assertEqual(wait_for_topics.wait(), True,
                         'cmd_vel topic not available')

        # Run the test node for seconds to accumulate data
        start_time = self.node.get_clock().now().nanoseconds / 1e9
        timeout = start_time + self.secs
        while (rclpy.ok() and
               self.node.get_clock().now().nanoseconds / 1e9 < timeout):
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if len(self.received_rates) >= self.minimum_messages:
                break

        # Check if the minimum number of messages were received
        self.assertEqual(len(self.received_rates) >= self.minimum_messages,
                         True,
                         'Insufficient messages received for 100 Hz rate')

        # # Verify if the publish rate is within the expected range
        received_rates = self.received_rates[int(len(self.received_rates)/2):]
        average_rate = sum(received_rates) / len(received_rates)
        self.assertEqual(average_rate < self.ref_rate+self.tolerance,
                         True, 'Publishing rate out of expected range')
        self.assertEqual(average_rate > self.ref_rate-self.tolerance,
                         True, 'Publishing rate out of expected range')
