from typing import Any, Dict, Protocol

import numpy as np


class Agent(Protocol):
    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        """Returns an action given an observation.

        Args:
            obs: observation from the environment.

        Returns:
            action: action to take on the environment.
        """
        raise NotImplementedError


class DummyAgent(Agent):
    def __init__(self, num_dofs: int):
        self.num_dofs = num_dofs

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        return np.zeros(self.num_dofs)


class BimanualAgent(Agent):
    def __init__(self, agent_left: Agent, agent_right: Agent):
        self.agent_left = agent_left
        self.agent_right = agent_right

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        left_obs = {}
        right_obs = {}
        for key, val in obs.items():
            L = val.shape[0]
            half_dim = L // 2
            assert L == half_dim * 2, f"{key} must be even, something is wrong"
            left_obs[key] = val[:half_dim]
            right_obs[key] = val[half_dim:]
        return np.concatenate(
            [self.agent_left.act(left_obs), self.agent_right.act(right_obs)]
        )
    
    def home(self, l_home_pos: np.ndarray, r_home_pos, hold: bool=False):
        '''Requires user to guide arm to specified position.
        Homing needs to be done once before other activities.
        A 30s timeout exists.

        :param l_home_pos: left arm's joint values (rad) to align to.
        :param r_home_pos: right arm's joint values (rad) to align to.
        :param hold: holds arm in position if calibration is successful  
        '''
        # validate input/state 
        if len(home_pos) != self._robot.num_dofs():
            raise ValueError(f"Expected {self._robot.num_dofs()} joints values, got {len(home_pos)}")
        if self._robot.is_calibrated():
            print("Already calibrated")
            return
        
        # calibrate
        calibrated = self._robot.calibrate(home_pos)
        if calibrated and hold:
            self._robot.set_position_control_mode()
            self._robot.set_torque_mode(True)   # TODO fix this functionality
    