""" 
This code teleoperates a simulated UR5e arm.
The code is modified from launch_nodes.py and run_env.py.

Expected result: the master and slave being in sync
"""

import numpy as np
from gello.env import RobotEnv
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.agents.gello_agent import GelloAgent
from gello.utils.control_utils import move_env_to_target_position

# set home position
joints_home = np.deg2rad([90, -90, 90, -90, -90, 0])
gello_port = "/dev/ttyUSB0"

def start_mujoco_server(port: int = 6001, host: str = "127.0.0.1"):
    """Starts a MujocoRobotServer in a background process. 
    A ZMQServerRobot is initialized in MujocoRobotServer.
    
    Args:
        robot_ip: IP address of the physical UR robot
        port: Communication port for the server (default: 6001)
        host: Network interface to bind to (default: "127.0.0.1")
    
    Returns:
        Process: Background process running the UR server"""

    import atexit
    import signal
    from multiprocessing import Process
    from pathlib import Path
    
    # use ur5e for arm and robotiq_2f85 for gripper
    PROJECT_ROOT : Path = Path(__file__).parent.parent.parent   
    MENAGERIE_ROOT : Path = PROJECT_ROOT / "third_party" / "mujoco_menagerie"   # mujoco_menagerie is external codebase containing 3D models # MuJoCo is a physics sim engine
    XML_DIR : Path = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"  # models are described with an xml file
    GRIPPER_XML_DIR : Path = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"

    # set up server for mujoco sim
    def _run_server():    
        from gello.robots.sim_robot import MujocoRobotServer        
        server = MujocoRobotServer(
            xml_path=str(XML_DIR),
            gripper_xml_path=str(GRIPPER_XML_DIR), 
            port=port, 
            host=host
        )
        server.serve()

    # run server as a background process 
    process = Process(target=_run_server)
    process.daemon = True
    process.start()
    
    # register cleanup
    def _cleanup():
        if process.is_alive():
            print("Stopping MuJoCo server...")
            process.terminate()
            process.join(timeout=1.0)
    atexit.register(_cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: _cleanup() or exit(0))
    
    return process
    
def main():
    # start mujoco server (contains ZMQServerRobot)
    start_mujoco_server()

    # create client
    robot_client = ZMQClientRobot() 
    env = RobotEnv(robot_client) 

    # create and connect to GELLO arm
    agent = GelloAgent(port=gello_port)

    # home sim arm
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
