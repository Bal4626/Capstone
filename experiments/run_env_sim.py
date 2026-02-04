import atexit
import signal
import time

from typing import Optional, Tuple

import numpy as np
from multiprocessing import Process

from gello.env import RobotEnv
from gello.zmq_core.robot_node import ZMQClientRobot

from gello.agents.gello_agent import GelloAgent

from pathlib import Path


DEFAULT_ROBOT_PORT : int = 6001
DEFAULT_ROBOT_HOST : str = "127.0.0.1"
CONTROL_RATE : int = 100

# MuJoCo is the physics engine
# Menagerie provides the 3D models/assets for MuJoCo
PROJECT_ROOT : Path = Path(__file__).parent.parent
MENAGERIE_ROOT : Path = PROJECT_ROOT / "third_party" / "mujoco_menagerie"

# Robot model - UR5e robot
XML_DIR : Path = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"

# Gripper model - robotiq_2f85
GRIPPER_XML_DIR : Path = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)

def start_mujoco_server(
    body_xml: Path,
    gripper_xml: Path, 
    port: int = 6001,
    host: str = "127.0.0.1"
):
    """
    Start MuJoCo simulation server in a background process.
    
    Args:
        body_xml: Path to XML file describing robotic arm model and meshes
        gripper_xml: Path to XML file describing gripper model and meshes
        port: Communication port for the simulation server (default: 6001)
        host: Network interface to bind to:
            - "127.0.0.1": Localhost only (default, most secure)
            - "0.0.0.0": All network interfaces (for remote connections)
            - Specific IP: Bind to particular network interface
    
    Returns:
        Process: Background process running the MuJoCo server
    """

    def _run_server():    
        from gello.robots.sim_robot import MujocoRobotServer
        server = MujocoRobotServer(
            xml_path=str(body_xml),
            gripper_xml_path=str(gripper_xml), 
            port=port, 
            host=host
        )
        server.serve()

    process = Process(target=_run_server)
    process.start()
    
    # Handle graceful exits
    def _cleanup():
        if process.is_alive():
            print("Stopping MuJoCo server...")
            process.terminate()
            process.join(timeout=1.0)
    
    atexit.register(_cleanup)
    
    # Handle Ctrl+C
    def _handle_sigint(sig, frame):
        print("\nInterrupted - stopping server...")
        _cleanup()
        exit(0)
    
    signal.signal(signal.SIGINT, _handle_sigint)

    return process
    
def main():
    # Starts simulation
    print("Starting MuJoCo simulation server...")
    start_mujoco_server(body_xml=XML_DIR,
                        gripper_xml=GRIPPER_XML_DIR,
                        port=DEFAULT_ROBOT_PORT,
                        host=DEFAULT_ROBOT_HOST)

    # To communicate/control simulation arm
    print("Setting up robot command executor...")       
    robot_client = ZMQClientRobot(port=DEFAULT_ROBOT_PORT,      # ZMQClientRobot: Low-level network pipe
                                  host=DEFAULT_ROBOT_HOST)
    env = RobotEnv(robot_client, control_rate_hz=CONTROL_RATE)      # RobotEnv: High-level interface with timing + structure

    # Connects to GELLO arm
    print("Setting up GELLO arm controller...")
    gello_port : str = "/dev/ttyUSB0"
    agent = GelloAgent(port=gello_port)

    # Set home position    
    home_joints = [180, -90, 90, -90, -90, 0]
    home_joint_gripper = home_joints.append(0)

    # Home sim arm
    env.step(np.deg2rad(np.array(home_joint_gripper))) 

    # Home GELLO arm
    agent.calibrate_arm(home_joints)

    # Run control loop
    obs = {}
    while True:
        action = agent.act(obs)
        obs = env.step(action)


if __name__ == "__main__":
    main()
