"""
This class is named GELLO agent but is just a high level representation of a Dynamixel arm.
It mainly interfaces with the RobotEnv class by receiving observation data and acting on it. 

Lower level implementation is in the DynamixelRobot class.
"""

import os
from typing import Dict, Optional, List
import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobotConfig
from gello.utils.kinematics import ScaledURKinematics

# arm specific settings
PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # ur5e at corner
    "/dev/ttyUSB0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_signs=(1, 1, -1, 1, 1, 1),
        servo_types=("XL330_M288",) * 6,  
        
        gripper_config=(7, 20, -30),
        kinematics=ScaledURKinematics("ur5e", 0.518)
    ),

    #ur3e left
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_signs=(1, 1, -1, 1, 1, 1),
        servo_types=("XL330_M288",) * 6,

        gripper_config=(7, 20, -30),
        kinematics=ScaledURKinematics("ur3e", 0.518)
    ),
    
    #ur3e right 
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_signs=(1, 1, -1, 1, 1, 1),
        servo_types=("XL330_M288",) * 6,

        gripper_config=(7, 20, -30),
        kinematics=ScaledURKinematics("ur3e", 0.518)
    )
}

class GelloAgent(Agent):
    """High level representation of a Dynamixel arm. Acts on observation data"""
    def __init__(self, port: str,
                 dynamixel_config: Optional[DynamixelRobotConfig]=None):
        '''Makes and connects to a dynamixel robot object using either predefined configs from port name or provided config.  

        :param port: Dynamixel controller specific port name (or just "/dev/ttyUSB0").         
        :param dynamixel_config: (Optional) The configuration of the physical robot for proper controls. 
        '''
        # validate inputs
        if os.path.exists(port) == False:
            raise RuntimeError(f"Port specified {port} not found among devices")
        if dynamixel_config is None:
            if port not in PORT_CONFIG_MAP:
                raise RuntimeError(f"Port {port} not in PORT_CONFIG_MAP and no dynamixel_config provided")
        
        # create robot object
        print(f"Using cached configurations for {port}" if dynamixel_config is None \
              else f"Using provided configurations {dynamixel_config}")
        config = PORT_CONFIG_MAP[port] if dynamixel_config is None else dynamixel_config 
        self._robot = config.make_robot(port=port)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """Acts on data in observations. 
        
        :return: arm joint angles and normalized gripper position (if any)."""
        
        # validate state
        if not self._robot.is_calibrated():
            raise RuntimeError("Arm not been homed. Home before use.")
        
        # apply force feedback
        if "wrench" in obs:
            wrench = obs["wrench"]
            self._robot.set_wrench(wrench)
        else:
            pass
            #print("alert: [wrench] key not in obs dict, cannot do perform force control")

        # apply gravity compensation 
        # TODO

        return self._robot.get_joint_state()
  
    
    def move(self, joint_state: np.ndarray):
        raise NotImplementedError
                         
    def home(self, home_pos: np.ndarray, hold: bool=False):
        '''Requires user to guide arm to specified position.
        Homing needs to be done once before other activities.
        A 30s timeout exists.

        :param home_pos: arm joint values (rad) to align to.
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
        