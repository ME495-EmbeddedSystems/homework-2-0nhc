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
This module contains tests for the physics simulation of the brick object in the virtual world.
"""
import random
import numpy as np
from turtle_brick.physics import World


# Set the number of experiments to test the function under random conditions.
NUM_EXP = 100


def generate_a_random_physics_engine():
    """
    Generate a randomized physics engine instance.

    Creates a `World` instance with random initial conditions for the brick position, platform radius,
    and timestep. Gravity is fixed at -9.81 m/s².

    Returns:
        tuple: A tuple containing:
            - `physics` (World): An instance of the World class with randomized parameters.
            - `brick` (list): Initial (x, y, z) location of the brick.
            - `gravity` (float): Gravitational acceleration (fixed at -9.81).
            - `radius` (float): Radius of the platform.
            - `dt` (float): Timestep for the physics simulation.
    """
    # Initialize the world randomly
    # -5m < x < 5m
    brick = [random.random()*10-5,
            random.random()*10-5,
            random.random()*10-5]
    gravity = -9.81  # fixed
    radius = random.random()*1.0 #  0-1m
    dt = random.random()*0.2 + 0.001 # 5hz to 1000hz
    physics = World(brick, gravity, radius, dt)
    
    return physics, brick, gravity, radius, dt


def test_brick_property():
    """
    Test that the `brick` property of the `World` instance matches the initial position.

    Runs `NUM_EXP` experiments, each time generating a random `World` instance and checking
    if the `brick` property correctly reflects the initial brick position.
    """
    for i in range(NUM_EXP):
        physics, brick, gravity, radius, dt = generate_a_random_physics_engine()
        assert physics.brick == brick


def test_brick_setter():
    """
    Test the `brick` setter of the `World` class.

    Runs `NUM_EXP` experiments, each time setting a new brick position in a random `World` instance.
    Verifies that:
        - The brick position is updated.
        - The time (`_t`) is reset to 0.
        - The velocities (`_xdot`, `_zdot`) are reset to 0.
    """
    for i in range(NUM_EXP):
        physics, brick, gravity, radius, dt = generate_a_random_physics_engine()
        physics.drop(z_limit=-np.inf)
        new_brick = [random.random()*10-5,
                    random.random()*10-5,
                    random.random()*10-5]
        physics.brick = new_brick
        assert physics.brick == new_brick
        assert physics._t == 0.0
        assert physics._xdot == 0.0
        assert physics._zdot == 0.0


def test_brick_drop():
    """
    Test the `drop` function of the `World` class with and without a z-axis limit.

    Runs `NUM_EXP` experiments where the brick is dropped under gravity, checking:
        1. Without a `z_limit`, ensuring the brick moves along the x, y, and z axes.
        2. With a `z_limit`, ensuring the brick movement stops at the specified limit on the z-axis.
    
    Verifies that the brick’s position updates correctly according to the calculated velocities and timestep.
    """
    for i in range(NUM_EXP):
        # First, test without the effect of z_limit
        physics, brick, gravity, radius, dt = generate_a_random_physics_engine()
        z_limit = -np.inf
        
        # generate a random pitch
        pitch = random.random()*np.pi - np.pi/2
        
        # Record the initial brick location
        x_cache = physics.brick[0]
        y_cache = physics.brick[1]
        z_cache = physics.brick[2]
        
        # Drop twice and the brick should move
        physics.drop(z_limit=z_limit, pitch=pitch)
        xdot_cache = physics._xdot
        ydot_cache = physics._ydot
        zdot_cache = physics._zdot
        physics.drop(z_limit=z_limit, pitch=pitch)
        
        assert physics._brick[0] == x_cache + xdot_cache * physics._dt
        assert physics._brick[1] == y_cache + ydot_cache * physics._dt
        assert physics._brick[2] == z_cache + zdot_cache * physics._dt
        
        # Then, test with the effect of z_limit
        physics, brick, gravity, radius, dt = generate_a_random_physics_engine()
        z_limit = brick[2]
        
        # generate a random pitch
        pitch = random.random()*np.pi - np.pi/2
        
        # Record the initial brick location
        x_cache = physics.brick[0]
        y_cache = physics.brick[1]
        z_cache = physics.brick[2]
        
        # Drop twice and the brick should move
        # The brick should stop in z because of z_limit
        physics.drop(z_limit=z_limit, pitch=pitch)
        xdot_cache = physics._xdot
        ydot_cache = physics._ydot
        zdot_cache = physics._zdot
        physics.drop(z_limit=z_limit, pitch=pitch)
        
        assert physics._brick[0] == x_cache + xdot_cache * physics._dt
        assert physics._brick[1] == y_cache + ydot_cache * physics._dt
        assert physics._brick[2] == z_limit
