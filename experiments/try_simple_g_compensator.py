import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class LinkProperties:
    """Properties for each link needed for gravity compensation"""
    mass: float  # kg
    com: np.ndarray  # Center of mass in link frame, shape (3,)
    inertia: Optional[np.ndarray] = None  # Inertia tensor in link frame (optional)


class GravityCompensator:
    """Simple gravity compensation using wrench projection method"""
    
    def __init__(self, kinematics, link_properties: List[LinkProperties]):
        """
        Args:
            kinematics: Kinematics model (e.g., ScaledURKinematics)
            link_properties: List of LinkProperties for each link
        """
        self._kin = kinematics
        self._links = link_properties
        
        # Gravity vector in world frame (default: z-down)
        self._g_world = np.array([0, 0, -9.81])
        
        if len(link_properties) != 6:
            raise ValueError(f"Expected 6 link properties for UR arm, got {len(link_properties)}")
    
    def set_gravity_vector(self, g_world: np.ndarray):
        """Set gravity vector in world frame (m/s²)"""
        if len(g_world) != 3:
            raise ValueError("Gravity vector must be 3D")
        self._g_world = g_world
    
    def compute_gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """
        Compute gravity compensation torque for given joint angles.
        
        Method: For each link, compute gravitational force in world frame,
        transform to link frame, compute resulting wrench at link frame,
        then project to joint space using Jacobian transpose.
        
        Returns: Gravity compensation torque (6,) in Nm
        """
        assert len(q) == 6
        
        # Get transformations to each link frame (excluding base and TCP)
        origins, transforms = self._kin.forward_kinematics(q)
        
        # We need transforms from world to each link frame (1-6)
        # transforms[0] = base (world), transforms[1] = link1, ..., transforms[6] = flange
        link_transforms = transforms[1:7]  # 6 link frames
        
        total_tau_gravity = np.zeros(6)
        
        for i in range(6):
            # Get transform from world to link i frame
            T_i = link_transforms[i]  # T_i: world -> link_i frame
            R_i = T_i[:3, :3]  # Rotation matrix: world -> link_i
            
            # 1. Compute gravitational force in world frame
            f_gravity_world = self._links[i].mass * self._g_world
            
            # 2. Transform force to link frame
            f_gravity_link = R_i.T @ f_gravity_world  # R_i^T = link_i -> world
            
            # 3. Create wrench in link frame
            # wrench = [force, torque] at link frame
            # Torque due to CoM offset: τ = r_com × f_gravity_link
            r_com = self._links[i].com  # CoM in link frame
            torque_link = np.cross(r_com, f_gravity_link)
            
            wrench_link = np.concatenate([f_gravity_link, torque_link])  # shape (6,)
            
            # 4. Transform wrench from link frame to world frame
            # For forces: f_world = R_i @ f_link
            # For torques: τ_world = R_i @ τ_link
            Ad_T_i = np.zeros((6, 6))
            Ad_T_i[:3, :3] = R_i
            Ad_T_i[3:, 3:] = R_i
            # Note: For pure translation, the wrench transformation would need
            # the [r]× terms, but since we're already at the link frame origin,
            # this simple rotation is sufficient.
            
            wrench_world = Ad_T_i @ wrench_link
            
            # 5. Compute required joint torque using Jacobian transpose
            # But we need Jacobian at the CoM location, not at TCP!
            # Let's compute the Jacobian at the link's CoM instead
            
            # First, compute CoM position in world frame
            p_com_world = T_i[:3, 3] + R_i @ r_com
            
            # Get Jacobian for point p_com_world (not TCP!)
            J_com_i = self._compute_jacobian_at_point(q, p_com_world, i+1)
            
            # 6. Project wrench to joint space: τ_i = J_com_iᵀ @ wrench_world
            tau_i = J_com_i.T @ wrench_world
            total_tau_gravity += tau_i
        
        return total_tau_gravity
    
    def _compute_jacobian_at_point(self, q: np.ndarray, point_world: np.ndarray, 
                                   up_to_link: int) -> np.ndarray:
        """
        Compute Jacobian for a specific point in world frame.
        
        Args:
            q: Joint angles
            point_world: Point position in world frame (3,)
            up_to_link: Compute Jacobian up to this link (1-indexed)
        
        Returns: Jacobian (6, up_to_link) for the point
        """
        origins, transforms = self._kin.forward_kinematics(q)
        
        # Jacobian columns
        J_v = []
        J_w = []
        
        for j in range(min(up_to_link, 6)):
            # Get z-axis of joint j in world frame
            # transforms[j+1] is frame after joint j
            T_j = transforms[j+1]
            z_axis = T_j[:3, 2]
            
            # Origin of joint j frame in world
            p_j = transforms[j+1][:3, 3]
            
            # Linear velocity Jacobian: z_axis × (p_point - p_j)
            Jv_j = np.cross(z_axis, point_world - p_j)
            
            # Angular velocity Jacobian: z_axis
            Jw_j = z_axis
            
            J_v.append(Jv_j)
            J_w.append(Jw_j)
        
        # Pad with zeros for unused joints
        while len(J_v) < 6:
            J_v.append(np.zeros(3))
            J_w.append(np.zeros(3))
        
        J = np.vstack([np.array(J_v).T, np.array(J_w).T])
        return J[:, :up_to_link]  # Only return columns up to requested link