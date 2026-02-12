from typing import Dict, Optional

import numpy as np

from gello.robots.robot import Robot

class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.20.25", no_gripper: bool = True):
        import rtde_control
        import rtde_receive

        try:
            print("Attempting to conenct to UR robot")
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
            self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        except Exception as e:
            print(e)
            print(robot_ip)

        if not no_gripper:
            from gello.robots.robotiq_gripper import RobotiqGripper

            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            # print("gripper connected")
            # self.gripper.activate()

        print("Connected to UR robot")

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

        #added this for haptic
        self._prev_gripper_pos = None
        self._prev_gripper_time = None
        self._gripper_velocity = 0.0
            
        #commented this to see if frame + joint sign fix feedback
        # self.robot.zeroFtSensor() 

        self._tau_ext_filtered = None
        self._alpha = 0.8  # 0 = more smoothing
        
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
        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints, velocity, acceleration, dt, lookahead_time, gain
        )
        if self._use_gripper:
            gripper_pos = joint_state[-1] * 255
            self.gripper.move(gripper_pos, 255, 10)
        self.robot.waitPeriod(t_start)

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

    def get_forces(self) -> np.ndarray:
        """Get the current TCP forces and torques from the UR5's wrist sensor in global frame.
        
        Returns:
            np.ndarray: 6-element array [Fx, Fy, Fz, Mx, My, Mz] in TCP frame. 
            If there is an error, [0, 0, 0, 0, 0, 0] is returned. 
        """
        try:
            # Get 6D wrench (in N and Nm)
            forces = np.array(self.r_inter.getActualTCPForce())
        except Exception as e:
            print(f"Error reading TCP force: {e}")
            forces = np.zeros(6)
        return forces


    def dh_transform(self,theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,     sa,     ca,    d],
            [0,     0,      0,    1]
        ])

    def ur5e_fk(self,q):
        # DH params for UR5e
        dh = [
            (q[0], 0.1625, 0.0, np.pi/2),
            (q[1], 0.0,   -0.425, 0.0),
            (q[2], 0.0,   -0.3922, 0.0),
            (q[3], 0.1333, 0.0, np.pi/2),
            (q[4], 0.0997, 0.0, -np.pi/2),
            (q[5], 0.0996, 0.0, 0.0)
        ]
        
        T = np.eye(4)
        Ts = [T.copy()]
        for theta, d, a, alpha in dh:
            A = self.dh_transform(theta, d, a, alpha)
            T = T @ A
            Ts.append(T.copy())
        return Ts  # list of T_0^0, T_0^1, ..., T_0^6

    def ur5e_jacobian_base(self,q):
        Ts = self.ur5e_fk(q)
        p_tcp = Ts[-1][:3, 3]
        
        J = np.zeros((6, 6))
        for i in range(6):  
            T_im1 = Ts[i]  # T_0^{i}
            z_im1 = T_im1[:3, 2]  # Z-axis of frame i (joint i+1 axis)
            p_im1 = T_im1[:3, 3]
            
            J[:3, i] = np.cross(z_im1, p_tcp - p_im1)
            J[3:, i] = z_im1
        return J

    def jacobian_tcp(self,q):
        J_base = self.ur5e_jacobian_base(q)
        T_06 = self.ur5e_fk(q)[-1]
        R = T_06[:3, :3]
        p = T_06[:3, 3]
        
        def skew(v):
            return np.array([[0, -v[2], v[1]],
                            [v[2], 0, -v[0]],
                            [-v[1], v[0], 0]])
        
        Ad_inv = np.block([
            [R.T, -R.T @ skew(p)],
            [np.zeros((3,3)), R.T]
        ])
        return Ad_inv @ J_base

    def get_external_joint_torques(self, q: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Estimate external joint torques from FT sensor.
        Valid under quasi-static conditions.
        """
        if q is None:
            q = self.get_joint_state()[:6]
        
        # Get wrench at TCP in tcp frame (correctly transformed)
        F_tcp = self.get_forces()  # [fx, fy, fz, mx, my, mz] in tcp
        
        # Get geometric Jacobian (6x6)
        J_tcp = self.jacobian_tcp(q)  # maps joint vel to spatial twist at TCP
        
        # External joint torques: τ = J^T @ F
        tau_ext = J_tcp.T @ F_tcp
        if self._tau_ext_filtered is None:
            self._tau_ext_filtered = tau_ext.copy()
        else:
            self._tau_ext_filtered = self._alpha * tau_ext + (1 - self._alpha) * self._tau_ext_filtered
        return self._tau_ext_filtered
    
    def get_joint_velocities(self):
        """Get the current velocites of the leader robot.

        Returns:
            T: The current velocities of the leader robot.
        """
        robot_joints_velocities = self.r_inter.getActualQd()
        if self._use_gripper:
            gripper_vel = 0 #set to 0 for now
            vel = np.append(robot_joints_velocities, gripper_vel)
        else:
            vel = robot_joints_velocities
        return vel 
    
    #test convert TCP force to base force
    def tcp_wrench_to_base(self,F_tcp, T_base_tcp):
        """
        Convert wrench from TCP frame to base frame.
        F_tcp: [Fx, Fy, Fz, Tx, Ty, Tz] in TCP frame
        T_base_tcp: 4x4 homogeneous transform from TCP to base
        Returns: F_base in base frame
        """
        R = T_base_tcp[:3, :3]
        p = T_base_tcp[:3, 3]

        def skew(v):
            return np.array([[0, -v[2], v[1]],
                            [v[2], 0, -v[0]],
                            [-v[1], v[0], 0]])

        F = F_tcp[:3]
        tau = F_tcp[3:]

        F_base = R @ F
        tau_base = R @ tau + skew(p) @ (R @ F)

        return np.hstack([F_base, tau_base])


    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        # print("ur5 joints:", joints) # we added this line
        pos_quat = np.zeros(7)

        # gripper_pos = np.array([joints[-1]])
        forces = self.get_forces()
        velocities = self.get_joint_velocities()
        
        return {
            "joint_positions": joints,
            "joint_velocities": velocities,
            "ee_pos_quat": pos_quat,
            "forces":forces
        }

def main():
    import time
    robot_ip = "192.168.20.25"
    ur = URRobot(robot_ip, no_gripper=False)
    
    ur.robot.zeroFtSensor()
    while True:
        q = ur.get_joint_state()[:6]
        F_tcp = ur.get_forces()
        q = q * (1, 1, 1, -1, 1, 1)

        # print(tcp_offset)
        print(f"Fx: {F_tcp[0]:5.2f}, Fy: {F_tcp[1]:5.2f}, Fz: {F_tcp[2]:5.2f}")
        print(f"τx: {F_tcp[0]:5.2f}, τy: {F_tcp[1]:5.2f}, τz: {F_tcp[2]:5.2f}")
        time.sleep(0.5)

if __name__ == "__main__":
    main()