# from typing import Dict

# import numpy as np

# from gello.robots.robot import Robot


# class URRobot(Robot):
#     """A class representing a UR robot."""

#     def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):
#         import rtde_control
#         import rtde_receive

#         robot_ip = "192.168.20.25"
#         self.in_force_mode = False

#         [print("in ur robot") for _ in range(4)]
#         try:
#             self.robot = rtde_control.RTDEControlInterface(robot_ip)
#         except Exception as e:
#             print(e)
#             print(robot_ip)

#         self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
#         if not no_gripper:
#             from gello.robots.robotiq_gripper import RobotiqGripper

#             self.gripper = RobotiqGripper()
#             self.gripper.connect(hostname=robot_ip, port=63352)
#             print("gripper connected")
#             # gripper.activate()

#         [print("connect") for _ in range(4)]

#         self._free_drive = False
#         self.robot.endFreedriveMode()
#         self._use_gripper = not no_gripper

#     def num_dofs(self) -> int:
#         """Get the number of joints of the robot.

#         Returns:
#             int: The number of joints of the robot.
#         """
#         if self._use_gripper:
#             return 7
#         return 6

#     def _get_gripper_pos(self) -> float:
#         import time

#         time.sleep(0.01)
#         gripper_pos = self.gripper.get_current_position()
#         assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
#         return gripper_pos / 255

#     def get_joint_state(self) -> np.ndarray:
#         """Get the current state of the leader robot.

#         Returns:
#             T: The current state of the leader robot.
#         """
#         robot_joints = self.r_inter.getActualQ()
#         if self._use_gripper:
#             gripper_pos = self._get_gripper_pos()
#             pos = np.append(robot_joints, gripper_pos)
#         else:
#             pos = robot_joints
#         return pos

#     def command_joint_state(self, joint_state: np.ndarray) -> None:
#         """Command the leader robot to a given state.

#         Args:
#             joint_state (np.ndarray): The state to command the leader robot to.
#         """
#         velocity = 0.5
#         acceleration = 0.5
#         dt = 1.0 / 500  # 2ms
#         lookahead_time = 0.2
#         gain = 100

#         robot_joints = joint_state[:6]
#         t_start = self.robot.initPeriod()
#         self.robot.servoJ(
#             robot_joints, velocity, acceleration, dt, lookahead_time, gain
#         )
#         if self._use_gripper:
#             gripper_pos = joint_state[-1] * 255
#             self.gripper.move(gripper_pos, 255, 10)
#         self.robot.waitPeriod(t_start)

#     def freedrive_enabled(self) -> bool:
#         """Check if the robot is in freedrive mode.

#         Returns:
#             bool: True if the robot is in freedrive mode, False otherwise.
#         """
#         return self._free_drive

#     def set_freedrive_mode(self, enable: bool) -> None:
#         """Set the freedrive mode of the robot.

#         Args:
#             enable (bool): True to enable freedrive mode, False to disable it.
#         """
#         if enable and not self._free_drive:
#             self._free_drive = True
#             self.robot.freedriveMode()
#         elif not enable and self._free_drive:
#             self._free_drive = False
#             self.robot.endFreedriveMode()

#     def get_observations(self) -> Dict[str, np.ndarray]:
#         joints = self.get_joint_state()
#         print("ur5 joints:", joints) # we added this line
#         pos_quat = np.zeros(7)
#         gripper_pos = np.array([joints[-1]])
#         return {
#             "joint_positions": joints,
#             "joint_velocities": joints,
#             "ee_pos_quat": pos_quat,
#             "gripper_position": gripper_pos,
#         }
    
#     def start_force_mode(
#         self,
#         task_frame=None,
#         selection_vector=None,
#         wrench=None,
#         force_type: int = 2,
#         limits=None,
#     ):
#         """
#         Wraps UR RTDEControlInterface.forceMode.

#         - task_frame: pose of the virtual force frame [x,y,z,rx,ry,rz] in base.
#         - selection_vector: which DOFs are compliant (1) or fixed (0).
#         - wrench: desired force/torque in the task frame.
#         - force_type: usually 2 for force control in base frame (check UR docs).
#         - limits: safety limits for motion in compliant axes.
#         """
#         if self.in_force_mode:
#             return

#         if task_frame is None:
#             task_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # base frame
#         if selection_vector is None:
#             # Example: only allow motion along z, keep others stiff
#             selection_vector = [0, 0, 1, 0, 0, 0]
#         if wrench is None:
#             # target: “hold” against contact with 0N (pure compliance)
#             wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         if limits is None:
#             # small safe range (meters/radians) – tune this!
#             limits = [0.05, 0.05, 0.05, 0.17, 0.17, 0.17]

#         self.robot.forceMode(task_frame, selection_vector, wrench, force_type, limits)
#         self.in_force_mode = True

#     def stop_force_mode(self):
#         if not self.in_force_mode:
#             return
#         self.robot.forceModeStop()
#         self.in_force_mode = False

    
#     def get_tcp_force(self):
#         """
#         Returns the measured TCP wrench as a numpy array [Fx, Fy, Fz, Tx, Ty, Tz] in N / Nm.
#         """
#         import numpy as np
#         wrench = self.r_inter.getActualTCPForce()
#         return np.array(wrench, dtype=float)



# def main():
#     robot_ip = "192.168.1.11"
#     ur = URRobot(robot_ip, no_gripper=True)
#     print(ur)
#     ur.set_freedrive_mode(True)
#     print(ur.get_observations())


# if __name__ == "__main__":
#     main()


from typing import Dict

import numpy as np

from gello.robots.robot import Robot


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):
        import rtde_control
        import rtde_receive

        # Hard-coded IP for your setup
        robot_ip = "192.168.20.25"

        self.in_force_mode = False
        self._free_drive = False
        self._use_gripper = not no_gripper

        [print("in ur robot") for _ in range(4)]
        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
        except Exception as e:
            print(e)
            print(robot_ip)

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)

        if not no_gripper:
            from gello.robots.robotiq_gripper import RobotiqGripper

            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            print("gripper connected")
            # self.gripper.activate()

        [print("connect") for _ in range(4)]

        # Make sure we start with freedrive off
        self.robot.endFreedriveMode()

        # Force-mode thresholds (N) – tune these for your setup
        self.force_enter_threshold = 15.0  # enter force mode above this
        self.force_exit_threshold = 8.0    # exit force mode when below this

    # --------------------------------------------------------------------- #
    # Basic robot properties
    # --------------------------------------------------------------------- #

    def num_dofs(self) -> int:
        """Get the number of joints of the robot."""
        if self._use_gripper:
            return 7
        return 6

    # --------------------------------------------------------------------- #
    # Gripper helpers
    # --------------------------------------------------------------------- #

    def _get_gripper_pos(self) -> float:
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    # --------------------------------------------------------------------- #
    # State access
    # --------------------------------------------------------------------- #

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the robot (joints + optional gripper)."""
        robot_joints = self.r_inter.getActualQ()
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    def get_tcp_force(self) -> np.ndarray:
        """
        Returns the measured TCP wrench as a numpy array [Fx, Fy, Fz, Tx, Ty, Tz]
        in N / Nm.
        """
        wrench = self.r_inter.getActualTCPForce()
        return np.array(wrench, dtype=float)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        print("ur5 joints:", joints)  # debug print you added
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }

    # --------------------------------------------------------------------- #
    # Commanding
    # --------------------------------------------------------------------- #

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """
        Command the robot to a given joint state using servoJ.
        Includes auto switching to / from force mode based on TCP force.
        """
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500  # 2 ms
        lookahead_time = 0.2
        gain = 100

        # ---- Auto force-mode switching based on measured TCP force --------
        wrench = self.get_tcp_force()
        f_mag = np.linalg.norm(wrench[:3])  # translational force magnitude

        if (not self.in_force_mode) and (f_mag > self.force_enter_threshold):
            print(f"[URRobot] Entering FORCE MODE (|F|={f_mag:.2f} N)")
            self.start_force_mode()

        elif self.in_force_mode and (f_mag < self.force_exit_threshold):
            print(f"[URRobot] Exiting FORCE MODE (|F|={f_mag:.2f} N)")
            self.stop_force_mode()
        # -------------------------------------------------------------------

        robot_joints = joint_state[:6]

        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints, velocity, acceleration, dt, lookahead_time, gain
        )

        if self._use_gripper:
            gripper_pos = joint_state[-1] * 255
            self.gripper.move(gripper_pos, 255, 10)

        self.robot.waitPeriod(t_start)

    # --------------------------------------------------------------------- #
    # Freedrive
    # --------------------------------------------------------------------- #

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode."""
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot."""
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    # --------------------------------------------------------------------- #
    # Force mode wrappers
    # --------------------------------------------------------------------- #

    def start_force_mode(
        self,
        task_frame=None,
        selection_vector=None,
        wrench=None,
        force_type: int = 2,
        limits=None,
    ) -> None:
        """
        Wraps UR RTDEControlInterface.forceMode.

        - task_frame: pose of the virtual force frame [x,y,z,rx,ry,rz] in base.
        - selection_vector: which DOFs are compliant (1) or fixed (0).
        - wrench: desired force/torque in the task frame.
        - force_type: usually 2 for force control in base frame.
        - limits: safety limits for motion in compliant axes.
        """
        if self.in_force_mode:
            return

        if task_frame is None:
            task_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # base frame
        if selection_vector is None:
            # Example: only allow motion along z, keep others stiff
            selection_vector = [0, 0, 1, 0, 0, 0]
        if wrench is None:
            # target: “hold” against contact with 0 N (pure compliance)
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if limits is None:
            # small safe range (meters/radians) – tune this!
            limits = [0.05, 0.05, 0.05, 0.17, 0.17, 0.17]

        self.robot.forceMode(task_frame, selection_vector, wrench, force_type, limits)
        self.in_force_mode = True

    def stop_force_mode(self) -> None:
        """Stop UR force mode if it is active."""
        if not self.in_force_mode:
            return
        try:
            self.robot.forceModeStop()
        except Exception:
            # If the session is already closed, ignore
            pass
        self.in_force_mode = False

    # --------------------------------------------------------------------- #
    # Cleanup (similar idea to `def exit(rtde_c): rtde_c.forceModeStop()`)
    # --------------------------------------------------------------------- #

    def __del__(self):
        """Best-effort cleanup so force mode doesn't stay active on shutdown."""
        try:
            if self.in_force_mode:
                self.robot.forceModeStop()
        except Exception:
            pass


def main():
    robot_ip = "192.168.1.11"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
