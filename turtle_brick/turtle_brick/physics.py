class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self._brick = brick
        self._gravity = gravity
        self._radius = radius
        self._dt = dt
        
        self._zdot = 0
        self._t = 0
        

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """
        return self._brick


    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self._brick = location
        self._zdot = 0.0
        self._t = 0.0


    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        self._brick[2] += self._zdot * self._dt
        self._zdot += self._gravity * self._dt
        if(self._brick[2] < 0.0):
            self._brick[2] = 0.0
            self._zdot = 0.0
        self._t += self._dt
        
        return self._brick