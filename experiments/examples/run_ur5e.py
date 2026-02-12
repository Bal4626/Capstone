"""
This code teleoperates a UR5e arm.
The code is modified from launch_nodes.py and run_env.py.

Expected result: the master and slave being in sync
"""

import numpy as np
from gello.env import RobotEnv
from gello.robots.ur import URRobot
from gello.zmq_core.robot_node import ZMQServerRobot, ZMQClientRobot
from gello.agents.gello_agent import GelloAgent
from gello.utils.control_utils import move_env_to_target_position

# robot ip (modify accordingly)
robot_ip: str = "192.168.1.10"
joints_home = np.deg2rad([90, -90, 90, -90, -90, 0])
gello_port = "/dev/ttyUSB0"

def start_ur_server(robot: URRobot, port: int = 6001, host: str = "127.0.0.1"):
    """Starts a UR robot server in a background process.
    The server connects to a physical UR robot and exposes it via ZMQ.
    
    Args:
        port: Communication port for the server (default: 6001)
        host: Network interface to bind to (default: "127.0.0.1")
    
    Returns:
        Process: Background process running the UR server
    """
    from multiprocessing import Process
    import atexit
    import signal

    # set up ur server
    def _run_server():
        server = ZMQServerRobot(robot, port=port, host=host)
        print(f"UR server starting on {host}:{port} for robot at {robot_ip}")
        server.serve()

    # run server as a background process
    process = Process(target=_run_server)
    process.daemon = True
    process.start()
    
    # register cleanup
    def _cleanup():
        if process.is_alive():
            print("Stopping UR server...")
            process.terminate()
            process.join(timeout=1.0)    
    atexit.register(_cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: _cleanup() or exit(0))
    
    return process

def main():
    # start UR server
    robot = URRobot(robot_ip)
    start_ur_server(robot)
    
    # create client
    robot_client = ZMQClientRobot()
    env = RobotEnv(robot_client)

    # create and connect to GELLO arm
    agent = GelloAgent(port=gello_port)
    
    # home ur arm
    move_env_to_target_position(env, joints_home)

    # home GELLO arm
    agent.home(joints_home)
    
    # run control loop
    obs = env.get_obs()    
    while True:
        action = agent.act(obs)
        obs = env.step(action)


if __name__ == "__main__":
    main()