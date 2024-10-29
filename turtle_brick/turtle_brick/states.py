# Predifined states for turtle_robot.py
MOVING = 0
"""int: The state where the turtle robot is actively moving toward a target."""

STOPPED = 1
"""int: The state where the turtle robot is stationary, without active movement."""

REACHED = 2
"""int: The state where the turtle robot has reached its destination or target."""


# Predifined states for arena.py
INIT = 3
"""int: The initial state for arena objects, indicating a starting configuration."""

PLACED = 4
"""int: The state where an object has been placed in a specific location in the arena."""

DROPPING = 5
"""int: The state where an object is in the process of being lowered or released."""

DROPPED = 6
"""int: The state where an object has been released and is stationary."""

FALLING = 7
"""int: The state where an object is freely moving downward, typically due to gravity."""

SLIDING = 8
"""int: The state where an object is moving across a surface without lifting."""


# Predifined states for catcher.py
INIT = 3
"""int: The initial state for the catcher, indicating a starting configuration."""

CATCHING = 9
"""int: The state where the catcher is actively attempting to capture or hold an object."""

NONAVAILABLE = 10
"""int: The state where the catcher is unavailable for use, possibly due to being occupied."""

CATCHED = 11
"""int: The state where the catcher has successfully captured an object."""

CLEARING = 12
"""int: The state where the catcher is clearing or releasing any held object or preparing for the next action."""
