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

        # Reuse URRobot's DH and Jacobian logic (but don't connect to real robot)
        self._ur_kinematics = URRobotKinematics()

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

    def get_jacobian(self, q=None) -> np.ndarray:
        if q is None:
            q = self.get_joint_state()[:6]
        return self._ur_kinematics.get_jacobian(q)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        gripper_pos = np.array([joints[-1]]) if self._use_gripper else np.array([0.0])
        return {
            "joint_positions": joints,
            "joint_velocities": np.zeros_like(joints),
            "ee_pos_quat": np.zeros(7),
            "gripper_position": gripper_pos,
            "forces": self.get_forces(),
        }

    # Optional: add dummy freedrive methods
    def set_freedrive_mode(self, enable: bool):
        pass

    def freedrive_enabled(self) -> bool:
        return False


class URRobotKinematics:
    """Pure kinematics (DH + Jacobian) without robot hardware dependency."""

    def get_jacobian(self, q: np.ndarray) -> np.ndarray:
        # ✅ Paste your DH-based Jacobian code here (from URRobot)
        DH_params = [
            (0,        np.pi/2, 0.1625, 0),
            (-0.425,   0,       0,      0),
            (-0.3922,  0,       0,      0),
            (0,        np.pi/2, 0.133,  0),
            (0,       -np.pi/2, 0.0997, 0),
            (0,        0,       0.0996, 0)
        ]

        n = len(q)
        T = np.eye(4)
        positions = [np.zeros(3)]
        z_axes = [np.array([0, 0, 1])]

        for i in range(n):
            a, alpha, d, theta_offset = DH_params[i]
            theta = q[i] + theta_offset
            Ti = np.array([
                [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0,              np.sin(alpha),                np.cos(alpha),                d],
                [0,              0,                            0,                            1]
            ])
            T = T @ Ti
            positions.append(T[:3, 3])
            z_axes.append(T[:3, 2])

        p_e = positions[-1]
        J_v = []
        J_w = []

        for i in range(n):
            Jv = np.cross(z_axes[i], p_e - positions[i])
            Jw = z_axes[i]
            J_v.append(Jv)
            J_w.append(Jw)

        J = np.vstack([np.array(J_v).T, np.array(J_w).T])
        return J