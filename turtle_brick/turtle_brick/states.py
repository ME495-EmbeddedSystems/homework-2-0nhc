#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""Predefined states for the turtle robot, arena, and catcher objects."""

# Predifined states for turtle_robot.py
MOVING = 0
"""
The state where the turtle robot is actively moving toward a target.
"""

STOPPED = 1
"""
The state where the turtle robot is stationary, without active movement.
"""

REACHED = 2
"""
The state where the turtle robot has reached its destination or target.
"""

# Predifined states for arena.py
INIT = 3
"""
The initial state for arena objects, indicating a starting configuration.
"""

PLACED = 4
"""
The state where an object has been placed in a specific location in the arena.
"""

DROPPING = 5
"""
The state where an object is in the process of being lowered or released.
"""

DROPPED = 6
"""
The state where an object has been released and is stationary.
"""

FALLING = 7
"""
The state where an object is freely moving downward, typically due to gravity.
"""

SLIDING = 8
"""
The state where an object is moving across a surface without lifting.
"""

# Predifined states for catcher.py
INIT = 3
"""
The initial state for the catcher, indicating a starting configuration.
"""

CATCHING = 9
"""
The state where the catcher is actively attempting to capture or hold an
object.
"""

NONAVAILABLE = 10
"""
The state where the catcher is unavailable for use, possibly due to being
occupied.
"""

CATCHED = 11
"""
The state where the catcher has successfully captured an object.
"""

CLEARING = 12
"""
The state where the catcher is clearing or releasing any held object or
preparing for the next action.
"""
