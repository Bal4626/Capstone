import numpy as np
from gello.env import RobotEnv
from gello.robots.ur import URRobot
from gello.robots.robot import BimanualRobot
from gello.zmq_core.robot_node import ZMQServerRobot, ZMQClientRobot
from gello.agents.gello_agent import GelloAgent
from gello.agents.agent import BimanualAgent
from gello.utils.control_utils import move_env_to_target_position

# robot ips (modify accordingly)
ROBOT_IP_LEFT: str = "192.168.2.10"
ROBOT_IP_RIGHT: str = "192.168.1.10"

# Home positions for bimanual UR arms
HOME_POS_LEFT = np.deg2rad([-90, -90, -90, -90, 90, 0])
HOME_POS_RIGHT = np.deg2rad([90, -90, 90, -90, -90, 0])
HOME_POS_BIMANUAL = np.concatenate([HOME_POS_LEFT, HOME_POS_RIGHT])

# GELLO ports for bimanual setup
GELLO_PORT_LEFT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0"
GELLO_PORT_RIGHT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0"

def start_bimanual_ur_server(
    robot_left: URRobot, 
    robot_right: URRobot, 
    port: int = 6001, 
    host: str = "127.0.0.1"
):
    """Starts a bimanual UR robot server in a background process.
    The server connects to two physical UR robots and exposes them via ZMQ as a bimanual system.
    
    Args:
        robot_left: Left URRobot instance
        robot_right: Right URRobot instance
        port: Communication port for the server (default: 6001)
        host: Network interface to bind to (default: "127.0.0.1")
    
    Returns:
        Process: Background process running the UR server
    """
    from multiprocessing import Process
    import atexit
    import signal

    # set up bimanual ur server
    def _run_server():
        bimanual_robot = BimanualRobot(robot_left, robot_right)
        server = ZMQServerRobot(bimanual_robot, port=port, host=host)
        print(f"Bimanual UR server starting on {host}:{port}")
        print(f"  Left robot IP: {ROBOT_IP_LEFT}")
        print(f"  Right robot IP: {ROBOT_IP_RIGHT}")
        server.serve()

    # run server as a background process
    process = Process(target=_run_server)
    process.daemon = True
    process.start()
    
    # register cleanup
    def _cleanup():
        if process.is_alive():
            print("Stopping bimanual UR server...")
            process.terminate()
            process.join(timeout=1.0)    
    atexit.register(_cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: _cleanup() or exit(0))
    
    return process

def main():
    # create left and right robot instances
    robot_left = URRobot(ROBOT_IP_LEFT)
    robot_right = URRobot(ROBOT_IP_RIGHT)
    
    # start bimanual UR server
    start_bimanual_ur_server(robot_left, robot_right)   # default port and host
    
    # create client
    robot_client = ZMQClientRobot() # default port and host
    env = RobotEnv(robot_client)
    
    # create bimanual GELLO agent
    agent_left = GelloAgent(port=GELLO_PORT_LEFT)
    agent_right = GelloAgent(port=GELLO_PORT_RIGHT)
    agent = BimanualAgent(agent_left, agent_right)

    # home gello arms
    agent_left.home(HOME_POS_LEFT)
    agent_right.home(HOME_POS_RIGHT)

    # home bimanual arms
    move_env_to_target_position(env, HOME_POS_BIMANUAL)
    
    # run control loop
    obs = env.get_obs()    
    while True:
        action = agent.act(obs)
        obs = env.step(action)

if __name__ == "__main__":
    main()