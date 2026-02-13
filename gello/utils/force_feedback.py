"""Force feedback utilities for robot teleoperation."""

import numpy as np
from typing import Dict, Optional, Tuple


def get_arm_torques(obs: Dict[str, np.ndarray], arm_index: int = 0, total_arms: int = 1) -> np.ndarray:
    """Extract torques for a specific arm from observations.
    
    Args:
        obs: Observations dictionary containing "torques"
        arm_index: Which arm to get torques for (0 for left/single, 1 for right in bimanual)
        total_arms: Total number of arms (1 for single, 2 for bimanual)
        
    Returns:
        np.ndarray: Torques for the specified arm (6 elements)
    """
    if "torques" not in obs:
        raise ValueError("No 'torques' key in observations")
    
    all_torques = obs["torques"]
    
    if total_arms == 1:
        # Single arm - return all torques (should be 6 elements)
        if len(all_torques) != 6:
            print(f"⚠️ Warning: Expected 6 torques for single arm, got {len(all_torques)}")
        return all_torques[:6]  # Take first 6 elements
    elif total_arms == 2:
        # Bimanual - split torques
        half_len = len(all_torques) // 2
        if arm_index == 0:
            return all_torques[:half_len][:6]  # Left arm torques
        else:
            return all_torques[half_len:][:6]  # Right arm torques
    else:
        raise ValueError(f"Unsupported number of arms: {total_arms}")

def force_feedback(agent, obs, arm_index: int = 0, total_arms: int = 1):
    """Apply force feedback to a single arm.
    
    Args:
        agent: The agent controlling the arm
        obs: Current observations (contains torques for ALL arms)
        arm_index: Which arm this is (0 for left/single, 1 for right in bimanual)
        total_arms: Total number of arms (1 for single, 2 for bimanual)
    """
    # Get joint signs from agent
    if not hasattr(agent, 'joint_signs'):
        print(f"⚠️ Agent doesn't have joint_signs attribute")
        return
    
    # Get torques for this specific arm
    try:
        arm_torques = get_arm_torques(obs, arm_index, total_arms)
    except ValueError as e:
        print(f"⚠️ Error getting torques: {e}")
        return
    
    # Check dimension match
    if len(arm_torques) != len(agent.joint_signs):
        print(f"⚠️ Dimension mismatch: torques ({len(arm_torques)}) != joint_signs ({len(agent.joint_signs)})")
        return
    
    haptic_gain = 1  # set to 1 as the downscaling is done on external force in ur.py
    
    scaled_torques = haptic_gain * arm_torques * agent.joint_signs
    scaled_torques = np.clip(scaled_torques, -0.08, 0.08)  # Nm
    
    arm_name = "Left" if arm_index == 0 else "Right" if total_arms == 2 else "Single"
    print(f"{arm_name} scaled_torques: {np.round(scaled_torques, 3)}")
    
    # Send torque value to GELLO (6 joints; gripper appended)
    if hasattr(agent, 'set_agent_torque'):
        try:
            # Append 0 torque for gripper if there is gripper
            if len(agent._robot._joint_ids) > 6:
                torque_command = np.append(scaled_torques, 0)
            agent.set_agent_torque(torque_command)  # type: ignore
        except Exception as e:
            print(f"⚠️ Torque command failed: {e}")