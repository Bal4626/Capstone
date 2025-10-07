# gello/agents/teleop_agent.py
import numpy as np
from gello.agents.agent import Agent

class TeleopAgent(Agent):
    """Keyboard joint-jog: returns TARGET JOINT POSITIONS."""
    def __init__(self, num_dofs=7, step=0.03, clip=0.4):
        self.num_dofs = num_dofs
        self.step = step
        self.clip = clip
        self.keys_down = set()
        self.q_target = None  # will be initialized from obs

    def act(self, obs):
        q = np.asarray(obs["joint_positions"], float)
        if self.q_target is None:
            self.q_target = q.copy()

        pairs = [('q','a'),('w','s'),('e','d'),('r','f'),('t','g'),('y','h'),('u','j')]
        for j,(plus,minus) in enumerate(pairs[:self.num_dofs]):
            if plus in self.keys_down:  self.q_target[j] += self.step
            if minus in self.keys_down: self.q_target[j] -= self.step

        # keep target near current joints
        self.q_target = np.clip(self.q_target, q - self.clip, q + self.clip)
        return self.q_target
