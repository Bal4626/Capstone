from typing import Tuple, List
import numpy as np

# DH Params
UR_MODEL_DH_MAP = {
    "ur3e": np.array([
            (0,      0.15185,  np.pi/2, 0),
            (-0.24355, 0,      0,       0),
            (-0.2132,  0,      0,       0),
            (0,      0.13105,  np.pi/2, 0),
            (0,      0.08535, -np.pi/2, 0),
            (0,      0.0921,   0,       0)
        ]),     

    "ur5e": np.array([
            (0,        0.1625,  np.pi/2, 0),
            (-0.425,   0,       0,       0),
            (-0.3922,  0,       0,       0),
            (0,        0.133,   np.pi/2, 0),
            (0,        0.0997, -np.pi/2, 0),
            (0,        0.0996,  0,       0)
        ])
}


class ScaledURKinematics:
    """This class handles the kinematics for a scaled UR arm."""

    def __init__(self, model: str, factor: float, tcp_DH: np.ndarray=np.array([(0, 0.05, 0, 0)])):
        '''
        Makes a kinematic model of a scaled UR arm.

        :param model: model name, e.g. "ur3e", "ur5e"
        :param factor: scaling factor to reach scaled arm
        :param tcp_DH: DH params to reach tool center point on scaled arm (standard).
        Format: [a[m], d[m], alpha[rad], theta[rad]]; Default 5 cm z axis offset
        '''
        assert factor > 0
        assert model in UR_MODEL_DH_MAP
        assert tcp_DH.shape == (1, 4)

        table = UR_MODEL_DH_MAP[model]
        self._tcp_DH = tcp_DH
        self._scaled_DH = self._scale_DH_params(factor, table)

    def _scale_DH_params(self, factor: float, DH_table: np.ndarray) -> np.ndarray:
        """ Scales the DH table for a scaled arm. """
        assert factor > 0
    
        scaled_DH = DH_table.copy()
        scaled_DH[:, 0] *= factor  # a: link length
        scaled_DH[:, 1] *= factor  # d: link offset

        return scaled_DH

    def _transform_matrix(self, a, d, alpha, theta) -> np.ndarray:
        """Returns the 4 by 4 transformation matrix (standard)."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,    sa,     ca,    d  ],
            [0,    0,      0,     1  ]
        ])
    
    def _forward_kinematics_flange(self, q: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Compute forward kinematics up to tool flange frame.
        
        Returns:
            origins: List of 7 frame origins (base + 6 frames)
            Ts: List of 7 transformation matrices """
        assert len(q) == 6
       
        # Setting params 
        n = len(q)    
        T = np.eye(4)
        Ts = [T.copy()]  # stores Ts
        origins = [np.zeros(3)]
        
        # Find origin and transform
        for i in range(n):
            a, d, alpha, theta_offset = self._scaled_DH[i]
            theta = q[i] + theta_offset
            
            Ti = self._transform_matrix(a, d, alpha, theta)  # Ti: frame i -> i-1
            T = T @ Ti      # T: frame i -> 0
            origins.append(T[:3, 3])
            Ts.append(T.copy())     # Ts: frame 0 -> 0, frame 1 -> 0, frame 2 -> 0,... 

        return origins, Ts
    
    def forward_kinematics(self, q: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Compute forward kinematics up to TCP.
            
        Returns:
            origins: List of 8 frame origins (base + 6 frames + TCP)
            Ts: List of 8 transformation matrices
        """
        assert len(q) == 6
        
        origins, Ts = self._forward_kinematics_flange(q)
        
        a, d, alpha, theta_offset = self._tcp_DH[0]
        Ti = self._transform_matrix(a, d, alpha, theta_offset)    # Ti: tcp frame -> flange
        T = Ts[-1]      # T: flange frame -> 0 
        T_tcp = T @ Ti     # T_tcp: tcp frame -> 0

        origins.append(T_tcp[:3, 3])
        Ts.append(T_tcp.copy())     # Ts: frame 0 -> 0, ..., frame 5 -> 0, tcp frame -> 0 

        return origins, Ts
    
    def get_jacobian_TCP(self, q: np.ndarray) -> np.ndarray:
        """Returns a 6 by 6 jacobian for the TCP in global frame."""
        assert len(q) == 6

        n = len(q) 
        origins, Ts = self.forward_kinematics(q)
        assert len(Ts) == 8

        # Select joint frame transforms and orgins
        Ts_j = Ts[1:7]  # T_select: frame 1 -> 0, frame 2 -> 0, ..., flange frame -> 0
        origins_j = origins[1:7]
        assert len(Ts_j) == 6
        assert len(origins_j) == 6

        # Construct Jacobian
        J_v = []
        J_w = []
        p_e = origins[-1] # tcp origin (global)
        z_axes = [Ts_j[i][:3, 2] for i in range(n)]  # [:3, 2]: z_axis
        
        for i in range(n):
            Jv = np.cross(z_axes[i], p_e - origins_j[i])
            Jw = z_axes[i]
            J_v.append(Jv)
            J_w.append(Jw)

        J = np.vstack([np.array(J_v).T, np.array(J_w).T])
        assert J.shape == (6, 6)
        
        return J