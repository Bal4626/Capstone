# gello/robots/simulated_ur5.py

from typing import Dict
import numpy as np
from gello.robots.robot import Robot
from gello.robots.ur import URRobot  # Reuse your DH + Jacobian logic

class SimulatedUR5(Robot):
    """Simulated UR5 for haptic testing with constant external force."""

    def __init__(
        self,
        start_joints=None,
        constant_force=None,  # e.g., [5.0, 0, 0, 0, 0, 0]
        use_gripper=False
    ):
        self._use_gripper = use_gripper
        dof = 7 if use_gripper else 6
        if start_joints is None:
            start_joints = np.deg2rad([0, -90, 90, -90, -90, 0])
            if use_gripper:
                start_joints = np.append(start_joints, 0.0)
        self._current_joints = np.array(start_joints, dtype=float)
        self._constant_force = np.array(constant_force, dtype=float) if constant_force is not None else np.zeros(6)


    def num_dofs(self) -> int:
        return 7 if self._use_gripper else 6

    def get_joint_state(self) -> np.ndarray:
        return self._current_joints.copy()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        # In simulation, just update internal state
        self._current_joints = np.array(joint_state, dtype=float)

    def get_forces(self) -> np.ndarray:
        # Return constant external force (e.g., 5N in X direction)
        return self._constant_force.copy()

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
    
    def get_external_joint_torques(self, q: np.ndarray = None) -> np.ndarray:
        """
        Estimate external joint torques from FT sensor.
        Valid under quasi-static conditions.
        """
        if q is None:
            q = self.get_joint_state()[:6]
        
        # Get wrench at TCP in base frame (correctly transformed)
        F_base = self.get_forces()  # [fx, fy, fz, mx, my, mz] in base
        
        # Get geometric Jacobian (6x6)
        # J_tcp = self.jacobian_tcp(q)  # maps joint vel to spatial twist at TCP
        J_base = self.ur5e_jacobian_base(q)
        # External joint torques: τ = J^T @ F
        tau_ext = J_base.T @ F_base

        #TCP force frame
        # F_tcp = self.get_forces()
        # Ts = self.ur5e_fk(q)
        # T_base_tcp = Ts[-1]
        # # Convert wrench from TCP frame to base frame
        # F_base = self.tcp_wrench_to_base(F_tcp, T_base_tcp)
        # print(F_base,"F_BASE")
        # # Get base-frame geometric Jacobian (6x6)
        # J_base = self.ur5e_jacobian_base(q)

        # # Compute joint torques: τ = J^T @ F
        # tau_ext = J_base.T @ F_base

        return tau_ext

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        gripper_pos = np.array([joints[-1]]) if self._use_gripper else np.array([0.0])
        forces = self.get_forces()
        torques = self.get_external_joint_torques()
        return {
            "joint_positions": joints,
            "joint_velocities": np.zeros_like(joints),
            "ee_pos_quat": np.zeros(7),
            "gripper_position": gripper_pos,
            "forces": forces,
            "torques":torques
        }

    # Optional: add dummy freedrive methods
    def set_freedrive_mode(self, enable: bool):
        pass

    def freedrive_enabled(self) -> bool:
        return False
    

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
    
