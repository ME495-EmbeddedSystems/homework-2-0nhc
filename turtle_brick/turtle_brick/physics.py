#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""This module contains the physics engine."""
import numpy as np


class World:
    """Class to simulate physics."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize with parameters.

        :param brick: The (x, y, z) location of the brick.
        :type brick: [float, float, float]
        :param gravity: The acceleration due to gravity in m/s^2.
        :type gravity: float
        :param radius: The radius of the platform.
        :type radius: float
        :param dt: Timestep in seconds for each physics simulation update.
        :type dt: float
        """
        self._brick = brick
        self._gravity = gravity
        self._radius = radius
        self._dt = dt

        self._xdot = 0.0
        self._ydot = 0.0
        self._zdot = 0.0
        self._t = 0.0

    @property
    def brick(self):
        """
        Get the current location of the brick.

        :return: The (x, y, z) location of the brick.
        :rtype: tuple
        """
        return self._brick

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location and reset its velocity.

        :param location: The new (x, y, z) location of the brick.
        :type location: tuple
        """
        self._brick = location
        self._xdot = 0.0
        self._zdot = 0.0
        self._t = 0.0

    def drop(self, z_limit=0.0, pitch=0.0, friction_dx=0.0):
        """
        Update the brick's location.

        :param z_limit: The lower limit on the z-axis for the brick's position.
        :type z_limit: float
        :param pitch: The pitch angle of the platform, affecting x-axis
                      acceleration.
        :type pitch: float
        :param friction_dx: Friction coefficient affecting horizontal (x-axis)
                            deceleration.
        :type friction_dx: float

        :return: Updated (x, y, z) location of the brick.
        :rtype: tuple
        """
        if (self._brick[2] <= z_limit):
            self._brick[0] += self._xdot * self._dt
            self._brick[2] = z_limit
            if (self._xdot > 0):
                xdot = self._xdot - friction_dx
                if (xdot > 0):
                    self._xdot = xdot
                else:
                    self._xdot = 0.0
            elif (self._xdot < 0):
                xdot = self._xdot + friction_dx
                if (xdot < 0):
                    self._xdot = xdot
                else:
                    self._xdot = 0.0
            else:
                self._xdot = 0.0
            self._zdot = 0.0
        else:
            self._brick[0] += self._xdot * self._dt
            self._brick[2] += self._zdot * self._dt

            self._xdot += self._gravity * self._dt * (-np.tan(pitch))
            self._zdot += self._gravity * self._dt

        if (self._brick[2] < z_limit):
            self._brick[2] = z_limit

        self._t += self._dt
        return self._brick
