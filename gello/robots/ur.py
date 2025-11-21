from typing import Dict

import numpy as np

from gello.robots.robot import Robot
import rtde_control
import rtde_receive
import dashboard_client
import time
from gello.robots.robotiq_gripper import RobotiqGripper



class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):


        robot_ip = "192.168.20.25"

        [print("in ur robot") for _ in range(4)]
        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
            self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
            self.dash = dashboard_client.DashboardClient(robot_ip)
            self.dash.connect()
        except Exception as e:
            print(e)
            print(robot_ip)


            
            
           

        if not no_gripper:
           

            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            print("gripper connected")
            # gripper.activate()

        [print("connect") for _ in range(4)]

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100

        robot_joints = joint_state[:6]
        print("aaa")
        t_start = self.robot.initPeriod()
        print("bbb")
        try:
            self.robot.servoJ(robot_joints, velocity, acceleration, dt, lookahead_time, gain)
        except:
            print("ServoJ failed")
        print("ccc")
        if self._use_gripper:
            gripper_pos = joint_state[-1] * 255
            self.gripper.move(gripper_pos, 255, 10)
        print("ddd")
        
        self.robot.waitPeriod(t_start)
        print("eee")

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def checkprotective_n_clear(self):
        test = self.r_inter.getSafetyStatusBits()
        print(f"{test:010b}")
        second_bit = (test >> 2) & 1
        print(second_bit)
        if second_bit == 1:
            print("Robot is in protective stop. Attempting to unlock...")
            time.sleep(1)
            self.dash.unlockProtectiveStop()
            time.sleep(1)
            print("Unlocked protective stop.")
            print(self.dash.getLoadedProgram())
            self.dash.stop()
            time.sleep(1)
            self.dash.play()
            time.sleep(1)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        # print("ur5 joints:", joints) # we added this line
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        self.checkprotective_n_clear()
    
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }
    


def main():
    robot_ip = "192.168.1.11"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())



if __name__ == "__main__":
    main()
