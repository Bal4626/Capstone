import os
from typing import Dict, Optional, List
import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot, DynamixelRobotConfig
from gello.utils.kinematics import ScaledURKinematics

PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    "/dev/ttyUSB0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_signs=(1, 1, -1, 1, 1, 1),
        servo_types=("XL330_M288",) * 6,  
        #gripper_config=None
        gripper_config=(7, 20, -30),
        kinematics=ScaledURKinematics("ur5e", 0.518)
    )
}

class GelloAgent(Agent):
    """
    Handles connection TCP re
    Handles the force control
    
    """
    def __init__(self, port: str,
                 dynamixel_config: Optional[DynamixelRobotConfig]=None):
        '''
        Makes a dynamixel robot object using specified port or specified config.  

        :param port: Dynamixel controller specific port name (or just "/dev/ttyUSB0").         
        :param dynamixel_config: (Optional) The configuration of the physical robot for proper controls. 
        '''
   
        assert os.path.exists(port), f"Port {port} not found among devices"
        assert port in PORT_CONFIG_MAP or dynamixel_config is not None, f"Port {port} not in config map"
        
        print(f"Using cached configurations for {port}" if dynamixel_config is None \
              else f"Using provided configurations {dynamixel_config}")
        
        config : DynamixelRobotConfig = PORT_CONFIG_MAP[port] if dynamixel_config is None else dynamixel_config 
        
        self._robot : DynamixelRobot = config.make_robot(port=port)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """Returns joint positions and normalized gripper position (if any)."""

        if "wrench" in obs:
            wrench = obs["wrench"]
            print(f"got {wrench}")
        else:
            print("alert: [wrench] key not in obs dict, cannot do perform force control")
        return self._robot.get_joint_state()
    
    def move(self, joint_state: np.ndarray):
        """ DOES NOT WORK. IN PROGRESS. Rotates joints"""
        assert joint_state.size == self._robot.num_dofs()
        
        self._robot.set_torque_mode(True)
        self._robot.command_joint_state(joint_state)
                
    def calibrate_arm(self, arm_pos: List[int, ]):
        '''Calibrates the arm when all arm joints are moved to the positions provided. A 30s timeout exists.

        :param arm_pos: arm joint values to align to, in degrees.
        '''
        if self._robot.is_calibrated():
            print("Already calibrated")
        else:
            self._robot.calibrate(np.deg2rad(arm_pos))
        