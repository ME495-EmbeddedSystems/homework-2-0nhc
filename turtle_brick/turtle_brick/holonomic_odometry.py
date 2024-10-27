import numpy as np

from turtle_brick.states import MOVING, STOPPED


class HolonomicOdometry:
    """Holonomic Odometry for a robot's traveled distance in the XY plane.
    
    This odometry class is suitable for holonomic robots, which can move in any direction 
    in the plane without the need to rotate in the direction of motion.
    
    Attributes:
        _x: The robot's x position state [m]
        _y: The robot's y position state [m]
        _theta: The robot's orientation state [rad]
        _time: Traveled time [s]
        _mode: The robot's current mode [MOVING/STOPPED]
        _travelled_distance: The total distance the robot has traveled [m]
    """
    
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0, initial_time=0.0) -> None:
        """Initializes the Holonomic Odometry parameters
        
        Args:
            initial_x: The initial x position state of the robot.
            initial_y: The initial y position state of the robot.
            initial_theta: The initial orientation state of the robot in radians.
            initial_time: The initial traveled time.
        """
        self._x = initial_x
        self._y = initial_y
        self._theta = initial_theta
        self._time = initial_time
        self._mode = STOPPED
        self._travelled_distance = 0.0
    
    def update_odom(self, vx, vy, w, t):
        """Update odometry with new states
        
        Given the robot's velocity states and current time, update the odometry states.
        
        Args:
            vx: Current linear velocity in the X axis [m/s]
            vy: Current linear velocity in the Y axis [m/s]
            w: Current angular velocity around the Z axis [rad/s]
            t: Current time in seconds [s]
        
        Returns:
            None
        
        Raises:
            None
        """
        if self._mode == STOPPED:
            # Start the odometry if it was previously stopped
            self._mode = MOVING
            self._time = t
        elif self._mode == MOVING:
            # Calculate the time interval
            dt = t - self._time
            
            # Compute the incremental changes in position and orientation
            dx = vx * dt
            dy = vy * dt
            dtheta = w * dt
            ddistance = np.sqrt(dx**2 + dy**2)
            
            # Update odometry states
            self._x += dx
            self._y += dy
            self._theta += dtheta
            self._travelled_distance += ddistance
            self._time = t

    def clear_states(self):
        """Clear odometry states
        
        Resets the odometry states to the initial conditions. Useful when resetting after a loop or task.
        
        Args:
            None
        
        Returns:
            None
        
        Raises:
            None
        """
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._time = 0.0
        self._mode = STOPPED
        self._travelled_distance = 0.0
    