from typing import Dict

import numpy as np

from gello.robots.robot import Robot
import rtde_control
import rtde_receive
# import dashboard_client
from dashboard_client import DashboardClient
import time
from gello.robots.robotiq_gripper import RobotiqGripper
from gello.robots.digital_gripper import DigitalGripper
import traceback


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "", no_gripper: bool = False): #BALRAJ  Turn the gripper on or off. 
        self.robot_ip = robot_ip
        # Auto-detect gripper type based on IP address
        self.use_digital_gripper = robot_ip.endswith("65")  # 192.168.20.65 (left arm) uses digital gripper
        self.use_robotiq_gripper = robot_ip.endswith("66")  # 192.168.20.66 (right arm) uses robotiq gripper

        [print("in ur robot") for _ in range(4)]
        try:
            self.robot = rtde_control.RTDEControlInterface(self.robot_ip)
            self.r_inter = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.dash = DashboardClient(self.robot_ip)
            self.dash.connect()
        except Exception as e:
            print(e)
            print(self.robot_ip)
            
        # Initialize appropriate gripper based on IP
        if not no_gripper:
            try:
                if self.use_digital_gripper:
                    print(f"Initializing digital gripper for {robot_ip}")
                    self.gripper = DigitalGripper(robot_ip)
                    print("Digital gripper connected")
                elif self.use_robotiq_gripper:
                    print(f"Initializing Robotiq gripper for {robot_ip}")
                    self.gripper = RobotiqGripper()
                    self.gripper.connect(hostname=robot_ip, port=63352)
                    print("Robotiq gripper connected")
                else:
                    print(f"No gripper configuration for IP {robot_ip}")
                    self.gripper = None
            except Exception as e:
                print(f"Failed to initialize gripper for {robot_ip}: {e}")
                self.gripper = None

        [print("connect") for _ in range(4)]

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper and hasattr(self, 'gripper') and self.gripper is not None

    def _ensure_control(self) -> bool:
        """Ensure RTDEControlInterface exists and is running."""
        try:
            if getattr(self, "robot", None) is None:
                self.robot = rtde_control.RTDEControlInterface(self.robot_ip)
            return True
        except Exception as e:
            print("Failed to (re)create RTDE control:", e)
            self.robot = None
            return False

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
        # Ensure control object exists (may be None after failed recovery)
        if not self._ensure_control():
            return

        self.checkprotective_n_clear()

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
        try:
            bits = self.r_inter.getSafetyStatusBits()
            bit2 = (bits >> 2) & 1   # IS_PROTECTIVE_STOPPED
            print(f"{bits:010b}")
            print("bit2 =", bit2)

            if bit2 == 0:
                return False  # no protective stop

            # 1) Wait a bit after collision (UR requires ~5s)
            time.sleep(5)

            # 2) Clear popup + unlock
            try:
                self.dash.closeSafetyPopup()
                self.dash.unlockProtectiveStop()
            except Exception:
                # reconnect dashboard if needed
                try:
                    self.dash.disconnect()
                except Exception:
                    pass
                self.dash = DashboardClient(self.robot_ip)
                self.dash.connect()
                self.dash.closeSafetyPopup()
                self.dash.unlockProtectiveStop()

            # 3) Wait until safety bit clears (bounded)
            for _ in range(100):  # 10s max
                if not self.r_inter.isProtectiveStopped():
                    break
                time.sleep(0.1)

            # 4) Start the program again if not running (ExternalControl / last URP)
            if not self.dash.running():
                def recreate_control():
                    try:
                        if self.robot is not None:
                            self.robot.stopScript()
                    except Exception:
                        pass
                    time.sleep(0.2)
                    try:
                        self.robot = None
                    except Exception:
                        pass
                    self.robot = rtde_control.RTDEControlInterface(self.robot_ip)

            ok = self.robot.reuploadScript()
            print("Reupload:", ok)
            if not ok:
                print("Still no RTDE control script – need to press PLAY on pendant / check program.")
                return False
                # time.sleep(2)  # wait a bit before starting

                # try:
                #     result = self.dash.play()
                #     print("Dashboard play():", result)
                # except Exception as e:
                #     traceback.print_exc()
                #     print("Dashboard play failed:", e)
                #     print("Recreating control + dashboard objects and retrying play().")
                #     recreate_control()
                #     self.dash = DashboardClient(self.robot_ip)
                #     self.dash.connect()
                #     try:
                #         self.dash.closeSafetyPopup()
                #         self.dash.unlockProtectiveStop()
                #         time.sleep(0.5)
                #         result = self.dash.play()
                #         print("Dashboard play() retry:", result)
                #     except Exception as e2:
                #         print("Second play() failed:", e2)
                #         print("Manual pendant play may be required.")
                #         return False

            # Kick controller once
            q = self.r_inter.getActualQ()
            if self._ensure_control():
                try:
                    self.robot.servoJ(q, 0.1, 0.1, 1/500, 0.2, 100)
                except Exception as e:
                    print("Recovery servo failed:", e)
                    return False
            print("Recovery complete.")
            return True
        except Exception as e:
            traceback.print_exc()
            print("Protective stop auto-recovery failed:", e)
            return False

    
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
    print("Testing UR Robot")
    robot_ip = "192.168.20.25"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())



if __name__ == "__main__":
    main()
