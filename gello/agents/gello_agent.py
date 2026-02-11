import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot



from scripts.gello_get_offset import get_config, Args


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Optional[Tuple[int, int, int]] = None
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        
        servo_types = ["XL330_M288"] * 6
        # append if there is gripper
        if self.gripper_config:
            servo_types.append("XL330_M077")
        
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
            servo_types = servo_types
        )

args = Args()
config_dict = get_config(args)
gripper_offsets = config_dict.get("gripper_offsets")
gripper_config = (7, gripper_offsets[0], gripper_offsets[1]) if gripper_offsets else None

PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    #ur3e left labeled L
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config = gripper_config
    ),
    #ur3e right 
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config = gripper_config
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NNNU-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config = gripper_config
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5I37-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config = gripper_config
    )
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        # Ensure start_joints is a numpy array if provided
        if start_joints is not None and not isinstance(start_joints, np.ndarray):
            start_joints = np.array(start_joints)
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
            #store joint signs
            self.joint_signs = np.array(dynamixel_config.joint_signs)
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)
            #store joint signs
            self.joint_signs = np.array(config.joint_signs)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        return self._robot.get_joint_state()
    
    # #for initial calibrations to start positions
    # def calibrate_arm(self, arm_pos: List[int, ]):
    #     '''Calibrates the arm when all arm joints are moved to the positions provided. A 30s timeout exists.

    #     :param arm_pos: arm joint values to align to, in degrees.
    #     '''
    #     if self._robot.is_calibrated():
    #         print("Already calibrated")
    #     else:
    #         self._robot.calibrate(np.deg2rad(arm_pos))
    
    #added this for torque haptic feedback
    def set_agent_torque(self, torques: np.ndarray):
        """Send torque commands to GELLO."""
        self._robot._driver.set_torque(torques.tolist())