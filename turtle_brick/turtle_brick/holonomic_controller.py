import numpy as np


class HolonomicController:
    """A controller for holonomic robots to compute velocities needed to reach a goal position."""
    def __init__(self, max_vel, kp) -> None:
        """
        Initialize the HolonomicController with maximum velocity and proportional gain.

        Args:
            max_vel (float): The maximum velocity the robot can achieve.
            kp (float): Proportional gain for the angular velocity control.
        """
        self._max_vel = max_vel
        self._kp = kp
    

    def holonomic_control(self, current_state, goal):
        """
        Compute the velocities required to move the robot towards a goal.

        Args:
            current_state (tuple): The current state of the robot as (x, y, theta).
            goal (tuple): The goal position as (xg, yg).

        Returns:
            tuple: A tuple (vel_x, vel_y, vel_theta) representing the computed velocities 
                   along x, y, and theta to move towards the goal.
        """
        # Load args for programming convenience
        x = current_state[0]
        y = current_state[1]
        theta = current_state[2]
        xg = goal[0]
        yg = goal[1]

        # Compute the theta between the current state and the goal
        theta_to_goal = np.arctan2(yg - y, xg - x)
        
        # Compute the velocity in x, y, and theta
        vel_x = self._max_vel * np.cos(theta_to_goal)
        vel_y = self._max_vel * np.sin(theta_to_goal)
        vel_theta = self._kp * (theta_to_goal - theta)

        return vel_x, vel_y, theta_to_goal
