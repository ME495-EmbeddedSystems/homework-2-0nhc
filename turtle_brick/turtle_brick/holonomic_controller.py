import numpy as np


class HolonomicController:
    def __init__(self, max_vel) -> None:
        self._max_vel = max_vel
    

    def holonomic_control(self, current_state, goal):
        # Load args for programming convenience
        x = current_state[0]
        y = current_state[1]
        theta = current_state[2]
        xg = goal[0]
        yg = goal[1]

        # Compute the theta between the current state and the goal
        theta_to_goal = np.arctan2(yg - y, xg - x)

        return theta_to_goal, self._max_vel

