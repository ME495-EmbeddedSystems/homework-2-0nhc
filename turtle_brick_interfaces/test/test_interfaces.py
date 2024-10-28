import pytest

def test_interfaces():
    try:
        from turtle_brick_interfaces.msg import Tilt
        from turtle_brick_interfaces.srv import Place
    except ImportError as e:
        pytest.fail(f"Failed to import interfaces: {e}")