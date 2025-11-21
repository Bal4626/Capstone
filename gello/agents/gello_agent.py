
# import os
# from dataclasses import dataclass
# from typing import Dict, Optional, Sequence, Tuple

# import numpy as np

# from gello.agents.agent import Agent
# from gello.robots.dynamixel import DynamixelRobot



# from scripts.gello_get_offset import get_config, Args


# @dataclass
# class DynamixelRobotConfig:
#     joint_ids: Sequence[int]
#     """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

#     joint_offsets: Sequence[float]
#     """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

#     joint_signs: Sequence[int]
#     """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

#     This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
#     """

#     gripper_config: Tuple[int, int, int]
#     """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

#     def __post_init__(self):
#         assert len(self.joint_ids) == len(self.joint_offsets)
#         assert len(self.joint_ids) == len(self.joint_signs)

#     def make_robot(
#         self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
#     ) -> DynamixelRobot:
#         return DynamixelRobot(
#             joint_ids=self.joint_ids,
#             joint_offsets=list(self.joint_offsets),
#             real=True,
#             joint_signs=list(self.joint_signs),
#             port=port,
#             gripper_config=self.gripper_config,
#             start_joints=start_joints,
#         )

# args = Args()
# config_dict = get_config(args)
# print(config_dict)

# PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
#     # xArm
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6, 7),
#         joint_offsets=(
#             3 * np.pi / 2,
#             2 * np.pi / 2,
#             1 * np.pi / 2,
#             4 * np.pi / 2,
#             -2 * np.pi / 2 + 2 * np.pi,
#             3 * np.pi / 2,
#             4 * np.pi / 2,
#         ),
#         joint_signs=(1, -1, 1, 1, 1, -1, 1),
#         gripper_config=(8, 195, 152),
#     ),
#     # yam
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U4GA-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6),
#         joint_offsets=[
#             0 * np.pi,
#             2 * np.pi / 2,
#             4 * np.pi / 2,
#             6 * np.pi / 6,
#             5 * np.pi / 3,
#             2 * np.pi / 2,
#         ],
#         joint_signs=(1, -1, -1, -1, 1, 1),
#         gripper_config=(
#             7,
#             -30,
#             24,
#         ),  # Reversed: now starts open (-30) and closes on press (24)
#     ),
#     # Left UR
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6),
#         joint_offsets=(
#             0,
#             1 * np.pi / 2 + np.pi,
#             np.pi / 2 + 0 * np.pi,
#             0 * np.pi + np.pi / 2,
#             np.pi - 2 * np.pi / 2,
#             -1 * np.pi / 2 + 2 * np.pi,
#         ),
#         joint_signs=(1, 1, -1, 1, 1, 1),
#         gripper_config=(7, 20, -22),
#     ),
#     # Right UR
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6),
#         joint_offsets=(
#             np.pi + 0 * np.pi,
#             2 * np.pi + np.pi / 2,
#             2 * np.pi + np.pi / 2,
#             2 * np.pi + np.pi / 2,
#             1 * np.pi,
#             3 * np.pi / 2,
#         ),
#         joint_signs=(1, 1, -1, 1, 1, 1),
#         gripper_config=(7, 286, 248),
#     ),
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NNNU-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6),
#         joint_offsets=tuple(
#             i for i in config_dict["joint_offsets"]
#         ),
#         joint_signs=(1, 1, -1, 1, 1, 1),
#         gripper_config=(7, config_dict["gripper_offsets"][0],  config_dict["gripper_offsets"][1]),
#         #gripper_config=(7, 19, -23)
#     ),
#     "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5I37-if00-port0": DynamixelRobotConfig(
#         joint_ids=(1, 2, 3, 4, 5, 6),
#         joint_offsets=tuple(
#             i for i in config_dict["joint_offsets"]
#         ),
#         joint_signs=(1, 1, -1, 1, 1, 1),
#         gripper_config=(7, config_dict["gripper_offsets"][0],  config_dict["gripper_offsets"][1]),
#         #gripper_config=(7, 19, -22)
#     )
# }


# class GelloAgent(Agent):
#     def __init__(
#         self,
#         port: str,
#         dynamixel_config: Optional[DynamixelRobotConfig] = None,
#         start_joints: Optional[np.ndarray] = None,
#     ):
#         # Ensure start_joints is a numpy array if provided
#         if start_joints is not None and not isinstance(start_joints, np.ndarray):
#             start_joints = np.array(start_joints)
#         if dynamixel_config is not None:
#             self._robot = dynamixel_config.make_robot(
#                 port=port, start_joints=start_joints
#             )
#         else:
#             assert os.path.exists(port), port
#             assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

#             config = PORT_CONFIG_MAP[port]
#             self._robot = config.make_robot(port=port, start_joints=start_joints)

#     def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
#         return self._robot.get_joint_state()


import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot

# FIXED: use proper package path so it works when called via `python -m ...`
from scripts.gello_get_offset import get_config, Args


@dataclass
class DynamixelRobotConfig:
    """Configuration for a single Dynamixel-based GELLO master arm."""

    joint_ids: Sequence[int]
    # The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...).

    joint_offsets: Sequence[float]
    # The joint offsets of GELLO. There needs to be a joint offset for each
    # joint_id and it should be a multiple of pi/2.

    joint_signs: Sequence[int]
    # The joint signs of GELLO. One sign per joint_id; each is either +1 or -1.
    # This will be different for each arm design.

    gripper_config: Tuple[int, int, int]
    # Gripper config of GELLO: (gripper_joint_id,
    #                           degrees in open_position,
    #                           degrees in closed_position).

    def __post_init__(self) -> None:
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self,
        port: str = "/dev/ttyUSB0",
        start_joints: Optional[np.ndarray] = None,
    ) -> DynamixelRobot:
        """Instantiate a real DynamixelRobot from this config."""
        return DynamixelRobot(
            joint_ids=list(self.joint_ids),
            joint_offsets=list(self.joint_offsets),
            joint_signs=list(self.joint_signs),
            gripper_config=self.gripper_config,
            port=port,
            real=True,
            start_joints=start_joints,
        )


# Load joint/gripper offsets from the offset helper
_args = Args()
_config_dict = get_config(_args)

PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            4 * np.pi / 2,
            -2 * np.pi / 2 + 2 * np.pi,
            3 * np.pi / 2,
            4 * np.pi / 2,
        ),
        joint_signs=(1, -1, 1, 1, 1, -1, 1),
        gripper_config=(8, 195, 152),
    ),
    # yam
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U4GA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=[
            0 * np.pi,
            2 * np.pi / 2,
            4 * np.pi / 2,
            6 * np.pi / 6,
            5 * np.pi / 3,
            2 * np.pi / 2,
        ],
        joint_signs=(1, -1, -1, -1, 1, 1),
        # Reversed: now starts open (-30) and closes on press (24)
        gripper_config=(7, -30, 24),
    ),
    # Left UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            0,
            1 * np.pi / 2 + np.pi,
            np.pi / 2 + 0 * np.pi,
            0 * np.pi + np.pi / 2,
            np.pi - 2 * np.pi / 2,
            -1 * np.pi / 2 + 2 * np.pi,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),
    # Right UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            np.pi + 0 * np.pi,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            1 * np.pi,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 286, 248),
    ),
    # Custom arm using offsets from gello_get_offset (your current arm)
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NNNU-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in _config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(
            7,
            _config_dict["gripper_offsets"][0],
            _config_dict["gripper_offsets"][1],
        ),
    ),
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5I37-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=tuple(i for i in _config_dict["joint_offsets"]),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(
            7,
            _config_dict["gripper_offsets"][0],
            _config_dict["gripper_offsets"][1],
        ),
    ),
}


class GelloAgent(Agent):
    """
    GELLO teleoperation agent for the Dynamixel master arm.

    Given observations from the environment, this agent simply returns the
    current joint state of the Dynamixel master; the environment is responsible
    for mapping this to the slave robot (e.g. UR5e).
    """

    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ) -> None:
        # Ensure start_joints is a numpy array if provided
        if start_joints is not None and not isinstance(start_joints, np.ndarray):
            start_joints = np.array(start_joints)

        if dynamixel_config is not None:
            # Explicit config passed in
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            # Look up config from the static port map
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """
        Return the current joint state of the Dynamixel master arm.

        The environment will use this as the command for the slave robot.
        """
        return self._robot.get_joint_state()
