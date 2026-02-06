from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


# Patched Args defaults and bimanual wiring:
# - left arm ip -> 192.168.20.65, gripper_type = digital
# - right arm ip -> 192.168.20.66, gripper_type = robotiq

@dataclass
class Args:
    robot: str = "ur"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.20.66"
    use_gripper: bool = True
    left_gripper: str = "digital"
    right_gripper: str = "robotiq"

