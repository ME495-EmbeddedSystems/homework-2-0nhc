#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""Holonomic controller to compute velocities for a goal position."""
import numpy as np


class HolonomicController:
    """Holonomic controller to compute velocities for a goal position."""

    def __init__(self, max_vel, kp) -> None:
        """
        Initialize with maximum velocity and proportional gain.

        :param max_vel: The maximum velocity the robot can achieve.
        :type max_vel: float
        :param kp: Proportional gain for the angular velocity control.
        :type: float
        """
        self._max_vel = max_vel
        self._kp = kp

    def holonomic_control(self, current_state, goal):
        """
        Compute the velocities required to move the robot towards a goal.

        :param current_state: The current state of the robot (x, y, theta)
        :type current_state: [float, ffloat, float]
        :param goal: The goal position as (xg, yg).
        :type goal: [float, float]

        :return: (vel_x, vel_y, vel_theta) representing the computed
                 velocities along x, y, and theta to move towards the goal.
        :rtype: tuple
        """
        # Load args for programming convenience
        x = current_state[0]
        y = current_state[1]
        xg = goal[0]
        yg = goal[1]

        # Compute the theta between the current state and the goal
        theta_to_goal = np.arctan2(yg - y, xg - x)

        # Compute the velocity in x, y, and theta
        vel_x = self._max_vel * np.cos(theta_to_goal)
        vel_y = self._max_vel * np.sin(theta_to_goal)

        return vel_x, vel_y, theta_to_goal
