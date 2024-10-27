import numpy as np


class HolonomicController:
    def __init__(self, max_vel, kp) -> None:
        self._max_vel = max_vel
        self._kp = kp
    

    def holonomic_control(self, current_state, goal):
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
