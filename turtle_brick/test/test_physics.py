import random
from turtle_brick.physics import World


# Initialize the world randomly
# -5m < x < 5m
brick = [random.random()*10-5,
         random.random()*10-5,
         random.random()*10-5]
gravity = -9.81  # fixed
radius = random.random()*1.0 #  0-1m
dt = random.random()*0.2 + 0.001 # 5hz to 1000hz
physics = World(brick, gravity, radius, dt)


def test_brick_property(physics=physics):
    assert physics.brick == brick
