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
This module contains tests for the turtle_brick_interfaces package.
"""
import pytest


def test_interfaces():
    """
    Test that required interfaces are available in the turtle_brick_interfaces package.

    Attempts to import `Tilt` message and `Place` service from `turtle_brick_interfaces`.
    If the imports fail, the test fails with an error message.

    Raises:
        pytest.fail: If any required interface cannot be imported.
    """
    try:
        from turtle_brick_interfaces.msg import Tilt
        from turtle_brick_interfaces.srv import Place
    except ImportError as e:
        pytest.fail(f"Failed to import interfaces: {e}")