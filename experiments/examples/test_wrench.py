"""
This experiment runs sweeping wrench on the master arm.
Requires the base frame directions to be knowm.

Expected result: force feedback directions matches base frame directions during sweep. 
"""

import numpy as np
from gello.agents.gello_agent import GelloAgent
import time
from gello.robots.robot import PrintRobot
from gello.env import RobotEnv

def get_sweeping_wrench(t, axis_idx=0, max_force=10, max_torque=1, period=5):
    """Generate sweeping wrench for given axis.
    
    Args:
        t: Current time
        axis_idx: 0-5 (Fx,Fy,Fz,Tx,Ty,Tz)
        max_force: Max force in N for first 3 axes
        max_torque: Max torque in Nm for last 3 axes
        period: Sweep period in seconds
    """
    wrench = np.zeros(6)
    value = max_force if axis_idx < 3 else max_torque
    # Sweep from 0 to max and back
    sweep = value * abs(np.sin(2 * np.pi * t / period))
    wrench[axis_idx] = sweep
    return wrench, sweep

def main():
    # create mock robot and env
    robot_client = PrintRobot(6)
    env = RobotEnv(robot_client)

    # create and connect to GELLO arm  
    gello_port = "/dev/ttyUSB0"
    agent = GelloAgent(port=gello_port)
    joints_home = np.deg2rad([90, -90, 90, -90, -90, 0])
    agent.home(joints_home)

    # miscellanous
    obs = env.get_obs()
    axis_names = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    start_time = time.time()    
    print("Running sweeping wrench (base coordinates)")

    # run control loop
    while True:
        t = time.time() - start_time
        axis_idx = int(t // 10) % 6  # change axis every 10s
        wrench, value = get_sweeping_wrench(t, axis_idx)
        print(f"{axis_names[axis_idx]}: {value:6.2f}")
        
        action = agent.act(obs)
        obs["wrench"] = wrench
        obs = env.step(action)

if __name__ == "__main__":
    main()
