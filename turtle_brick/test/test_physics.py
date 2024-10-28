import random
import numpy as np
from turtle_brick.physics import World


# Set the number of experiments in order to test the function 
# under random conditions.
NUM_EXP = 100


def generate_a_random_physics_engine():
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
    for i in range(NUM_EXP):
        physics, brick, gravity, radius, dt = generate_a_random_physics_engine()
        assert physics.brick == brick


def test_brick_setter():
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
