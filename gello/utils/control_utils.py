"""Shared utilities for robot control loops."""

import datetime
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from gello.agents.agent import Agent
from gello.env import RobotEnv

DEFAULT_MAX_JOINT_DELTA = 1.0


def move_to_start_position(
    env: RobotEnv, agent: Agent, max_delta: float = 1.0, steps: int = 25
) -> bool:
    """Move follower robot to agent position gradually.

    Args:
        env: Robot environment
        agent: Agent that provides target position
        max_delta: Maximum joint delta per step
        steps: Number of steps for gradual movement

    Returns:
        bool: True if successful, False if position too far
    """
    print("Going to start position")
    start_pos = agent.act(env.get_obs())
    obs = env.get_obs()
    joints = obs["joint_positions"]

    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = DEFAULT_MAX_JOINT_DELTA
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print()
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return False

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    for _ in range(steps):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        current_joints = obs["joint_positions"]
        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)

    return True

def move_env_to_target_position(
    env: RobotEnv, target_position: np.ndarray, max_delta: float = 1.0, steps: int = 25
) -> None:
    """Move follower robot to target position gradually.

    Args:
        env: Robot environment
        target_position: Desired joint positions to move to
        max_delta: Maximum joint delta per step
        steps: Number of steps for gradual movement

    Raises:
        RuntimeError: If joint differences exceed DEFAULT_MAX_JOINT_DELTA
    """
    print("Moving robot to start position")
    obs = env.get_obs()
    joints = obs["joint_positions"]

    abs_deltas = np.abs(target_position - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    if abs_deltas[id_max_joint_delta] > DEFAULT_MAX_JOINT_DELTA:
        id_mask = abs_deltas > DEFAULT_MAX_JOINT_DELTA
        error_msg = "\nCannot move to start position - joint differences too large:\n"
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, target, current_j in zip(
            ids,
            abs_deltas[id_mask],
            target_position[id_mask],
            joints[id_mask],
        ):
            error_msg += (
                f"  joint[{i}]: delta: {delta:4.3f} (max: {DEFAULT_MAX_JOINT_DELTA}), "
                f"target: {target:4.3f}, current: {current_j:4.3f}\n"
            )
        raise RuntimeError(error_msg)

    print(f"Target pos: {len(target_position)}", f"Current joints: {len(joints)}")
    assert len(target_position) == len(joints), f"target dim = {len(target_position)}, but env dim = {len(joints)}"

    current_pos = joints.copy()
    for _ in range(steps):
        delta = target_position - current_pos
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        current_pos = current_pos + delta
        env.step(current_pos)

class SaveInterface:
    """Handles keyboard-based data saving interface."""

    def __init__(
        self,
        data_dir: str = "data",
        agent_name: str = "Agent",
        expand_user: bool = False,
    ):
        """Initialize save interface.

        Args:
            data_dir: Base directory for saving data
            agent_name: Name of agent (used for subdirectory)
            expand_user: Whether to expand ~ in data_dir path
        """
        from gello.data_utils.keyboard_interface import KBReset

        self.kb_interface = KBReset()
        self.data_dir = Path(data_dir).expanduser() if expand_user else Path(data_dir)
        self.agent_name = agent_name
        self.save_path: Optional[Path] = None

        print("Save interface enabled. Use keyboard controls:")
        print("  S: Start recording")
        print("  Q: Stop recording")

    def update(self, obs: Dict[str, Any], action: np.ndarray) -> Optional[str]:
        """Update save interface and handle saving.

        Args:
            obs: Current observations
            action: Current action

        Returns:
            Optional[str]: "quit" if user wants to exit, None otherwise
        """
        from gello.data_utils.format_obs import save_frame

        dt = datetime.datetime.now()
        state = self.kb_interface.update()

        if state == "start":
            dt_time = datetime.datetime.now()
            self.save_path = (
                self.data_dir / self.agent_name / dt_time.strftime("%m%d_%H%M%S")
            )
            self.save_path.mkdir(parents=True, exist_ok=True)
            print(f"Saving to {self.save_path}")
        elif state == "save":
            if self.save_path is not None:
                save_frame(self.save_path, dt, obs, action)
        elif state == "normal":
            self.save_path = None
        elif state == "quit":
            print("\nExiting.")
            return "quit"
        else:
            raise ValueError(f"Invalid state {state}")

        return None


def run_control_loop(
    env: RobotEnv,
    agent: Agent,
    save_interface: Optional[SaveInterface] = None,
    print_timing: bool = True,
    use_colors: bool = False,
) -> None:
    """Run the main control loop.

    Args:
        env: Robot environment
        agent: Agent for control
        save_interface: Optional save interface for data collection
        print_timing: Whether to print timing information
        use_colors: Whether to use colored terminal output
    """
    # Check if we can use colors
    colors_available = False
    colored = None  # Define colored in outer scope
    
    if use_colors:
        try:
            from termcolor import colored
            colors_available = True
            start_msg = colored("\nStart 🚀🚀🚀", color="green", attrs=["bold"])
        except ImportError:
            start_msg = "\nStart 🚀🚀🚀"
    else:
        start_msg = "\nStart 🚀🚀🚀"

    print(start_msg)

    start_time = time.time()
    obs = env.get_obs()

    while True:
        if print_timing:
            num = time.time() - start_time
            message = f"\rTime passed: {round(num, 2)}          "
        
            if colors_available and colored is not None:  # Check colored exists
                print(
                    colored(message, color="white", attrs=["bold"]), end="", flush=True
                )
            else:
                print(message, end="", flush=True)

        action = agent.act(obs)

        # Handle save interface
        if save_interface is not None:
            result = save_interface.update(obs, action)
            if result == "quit":
                break

        obs = env.step(action)