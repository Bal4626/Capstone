import time
import numpy as np
from typing import Any, Dict
from gello.agents.agent import Agent

class ForceControlAgent(Agent):
    """
    Hybrid: posture (move toward target) + tiny force feedback on +Z.
    Logs joints + wrench each tick.
    Expects to output JOINT VELOCITIES (rad/s). Adjust gains to your dt.
    """

    def __init__(self, num_dofs: int = 7, force_gain: float = 2.0,
                 post_gain: float = 0.5, log_path: str = "force_log.csv"):
        self.num_dofs = num_dofs
        self.force_gain = float(force_gain)   # [rad/(s·N)] approx scalar map
        self.post_gain  = float(post_gain)    # [1/s] proportional posture gain
        self.log_path = log_path
        self.start_time = time.time()

        # Target posture (ensure length matches DOFs)
        
        self.target = np.deg2rad([-100, 20, 80, -60, 50, 70, -40])
        print("Target (deg):", np.round(np.rad2deg(self.target), 1))

        # Create log file with header
        with open(self.log_path, "w") as f:
            header = (
                "time," + ",".join([f"q{i}" for i in range(self.num_dofs)]) +
                ",fx,fy,fz,tx,ty,tz\n"
            )
            f.write(header)

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        #print("Raw joint_positions:", obs["joint_positions"])

        # --- Read observations robustly ---
        qpos = np.asarray(obs.get("joint_positions", np.zeros(self.num_dofs)), dtype=float)
        if qpos.shape[0] != self.num_dofs:
            # pad or trim to match DOFs
            qpos = (np.pad(qpos, (0, max(0, self.num_dofs - qpos.shape[0])), constant_values=0.0)
                    [:self.num_dofs])

        wrench = np.asarray(obs.get("ee_force", np.zeros(6)), dtype=float)
        if wrench.shape[0] != 6:
            wrench = np.zeros(6, dtype=float)

        # --- Force feedback: try to feel +1N in z (fz) ---
        desired_wrench = np.array([0, 0, 1, 0, 0, 0], dtype=float)  # [N, N, N, Nm, Nm, Nm]
        force_error = desired_wrench - wrench  # 6D

        # Extremely simplified mapping: use only fz to bias elbow/wrist joints a hair.
        # NOTE: For real control, use Jᵀ * wrench or a cartesian impedance.
        fz = float(force_error[2])  # +z force error
        force_term = np.zeros(self.num_dofs, dtype=float)
        # Distribute a tiny amount to the last few joints so the arm "yields" up/down
        if self.num_dofs >= 3:
            force_term[-3:] = self.force_gain * fz / 3.0  # rad/s from N (very small)

        # --- Posture term: move toward target (as velocities) ---
        posture_term =  self.post_gain * (self.target - qpos)  # rad/s

        # --- Blend & clip joint velocities ---
        qvel_cmd = posture_term + force_term
        qvel_cmd = np.clip(qvel_cmd, -0.2, 0.2)  # rad/s safety

        # --- Log ---
        with open(self.log_path, "a") as f:
            t = time.time() - self.start_time
            row = (
                f"{t}," +
                ",".join(f"{x:.6f}" for x in qpos) + "," +
                ",".join(f"{x:.6f}" for x in wrench) + "\n"
            )
            f.write(row)

        # Return joint velocities (rad/s)
        #return qvel_cmd
        qpos_next = qpos + 0.1 * (self.target - qpos)
        return qpos_next
