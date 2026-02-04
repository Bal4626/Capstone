import numpy as np
from gello.agents.gello_agent import GelloAgent
from gello.utils.kinematics import ScaledURKinematics
import time


def main():
    #robot_ip = "192.168.20.25"
    
    gello_port : str = "/dev/ttyUSB0"
    agent = GelloAgent(port=gello_port)
    
    home_pos = [180, -90, 90, -90, -90, 0]
    agent.calibrate_arm(home_pos)
    
    while True:
        wrench_base = np.array((0, 0, -10, 0, 0, 0))

        #agent._robot.print_state()
        r = agent._robot.get_attentuated_torque_for_wrench(wrench_base)
        print(r)

        time.sleep(0.3)

if __name__ == "__main__":
    main()
