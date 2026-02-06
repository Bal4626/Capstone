from typing import Dict, Optional, Sequence, Tuple
import numpy as np
import time

from dataclasses import dataclass
from gello.robots.robot import Robot
from gello.dynamixel.driver import DynamixelDriver, FakeDynamixelDriver
from gello.utils.kinematics import ScaledURKinematics

class DynamixelRobot(Robot):
    """A class representing a Dynamixel robot. 
    
    Handles hardware-specific details: signs, offsets, gripper presence, etc."""
    def __init__(
        self,
        joint_ids: Tuple[int, ...],
        joint_signs: Sequence[int],
        servo_types: Sequence[str],
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        ):  
        """Initialize Dynamixel robot.
        
        :param joint_ids: List of Dynamixel motor IDs for the arm joints
        :param joint_signs: Direction multipliers (1 or -1) for each arm joint
        :param servo_types: List of servo models
        :param real: Whether to use real hardware or simulation
        :param port: Serial port for real hardware
        :param baudrate: Communication baud rate
        :param gripper_config: (Optional) tuple of (gripper_id, open_angle, close_angle) in degrees. 
        If not specified, will be initialized without a gripper
        :param servo_types: (Optional)[Sequence[str]] = None
        """
        assert len(joint_signs) == len(joint_ids), \
            "joint_signs and joint_ids length must match"
        
        assert len(joint_signs) == len(servo_types), \
            "joint_signs and servo_types length must match"
        
        assert np.all(np.abs(joint_signs) == 1), \
            "joint_signs must be 1 or -1"
        
        assert len(joint_signs) == len(joint_ids), \
            "joint_signs and joint_ids length must match"

        self._joint_ids = tuple(joint_ids)
        self._joint_offsets = np.zeros(len(joint_ids))
        self._joint_signs = np.array(joint_signs)
        self._has_gripper = False if gripper_config is None else True
        
        self._ids = tuple(joint_ids) if gripper_config is None \
                        else tuple(joint_ids) + (gripper_config[0],)

        self._gripper_open_close = None if gripper_config is None \
                        else (np.deg2rad(gripper_config[1]), np.deg2rad(gripper_config[2])) 
        
        self._driver = DynamixelDriver(self._ids, port=port, baudrate=baudrate, servo_types=servo_types) if real \
                        else FakeDynamixelDriver(self._ids)

        if len(self._driver.get_joints()) > self.num_actuators():
            raise RuntimeError("More actuators found then initialized for. " \
            "Ensure gripper_config is provided if gripper is present")
        
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99
        self._table_alr_printed = False
        self._is_calibrated = False

        self.set_current_control_mode()

    def num_dofs(self) -> int:
        """Return number of degrees of freedom of the robotic arm."""
        return len(self._joint_ids)

    def num_actuators(self) -> int:
        """Return number of actuators."""
        return self.num_dofs() + int(self.has_gripper())

    def has_gripper(self) -> bool:
        """Returns whether robot was initialized with gripper."""
        return self._has_gripper

    def is_calibrated(self) -> bool:
        """Returns whether the arm has been calibrated."""
        return self._is_calibrated

    def _smooth(self, positions) -> np.ndarray:
        """Apply exponential smoothing to positions."""
        if self._last_pos is None:
            self._last_pos = positions
            return positions
        else:
            smoothed_positions = self._last_pos * (1 - self._alpha) + positions * self._alpha
            self._last_pos = smoothed_positions
            return smoothed_positions

    def get_joint_state(self) -> np.ndarray:
        """Returns the positions of all actuators. 
        
        If gripper is present, the last element is normalized to [0,1] where 0 = fully open, 1 = fully closed."""
        readings = self._driver.get_joints()
        
        # Process arm
        arm_raw = readings[:self.num_dofs()]
        assert arm_raw.size == self.num_dofs()
        arm_calibrated = arm_raw * self._joint_signs + self._joint_offsets

        # Process gripper if present
        gripper_processed = None
        if self._has_gripper:
            gripper_raw = readings[-1]
            
            assert self._gripper_open_close is not None
            
            open_angle, close_angle = self._gripper_open_close
            gripper_norm = (gripper_raw - open_angle) / (close_angle - open_angle)
            gripper_processed = np.clip(gripper_norm, 0, 1)

        # Combine and return results
        positions = arm_calibrated if gripper_processed is None \
            else np.append(arm_calibrated, gripper_processed) 
        
        assert positions.size == self.num_actuators()

        return self._smooth(positions)

    def get_joint_state_arm(self) -> np.ndarray:
        """Returns the positions of arm actuators in radians."""
        arm_pos = self.get_joint_state()[:-1] if self._has_gripper else self.get_joint_state()
        assert arm_pos.size == self.num_dofs()
        return arm_pos
    
    def get_gripper_state(self) -> float | None:
        return self.get_joint_state()[-1] if self._has_gripper else None

    def command_joint_state(self, joint_state: np.ndarray):
        """Send position commands to all actuators.
        
        :param joint_state: Target positions in radians for arm joints,
                        with optional gripper value in [0,1] as last element.
        :type joint_state: np.ndarray         

        """
        assert len(joint_state) == len(self._ids), \
            f"Expected {len(self._ids)} values (arm + gripper), got {len(joint_state)}"

        if self._has_gripper:
            gripper_norm = joint_state[-1]
            
            assert gripper_norm >= 0 and gripper_norm <= 1, \
                "Normalized gripper value must be in [0,1]"
            
            assert len(joint_state[:-1]) == self.num_dofs(), \
                "Expected {self.num_dofs()} arm joints, got {len(joint_state[:-1])}"
        else:
            assert len(joint_state) == self.num_dofs(), \
                f"Expected {self.num_dofs()} arm joints, got {len(joint_state)}"
        
        # Process arm joints
        target_arm = joint_state[:self.num_dofs()]
        target_arm_raw = target_arm / self._joint_signs - self._joint_offsets
        
        targets = target_arm_raw.tolist()

        # Process gripper if present
        if self._has_gripper:
            target_gripper = joint_state[-1]
            open_angle, close_angle = self._gripper_open_close
            target_gripper_raw = target_gripper * (close_angle - open_angle) + open_angle
            targets.append(target_gripper_raw)

        self._driver.set_joints(targets)
    
    def _check_wrapped_error(self, pose_ref : np.ndarray) -> np.ndarray:
        """Returns the wrapped error between current arm pose and reference pose, in radians.

        :param pose_ref: Desired joint positions in radians
        :return: Wrapped error (pose_ref - current_position) in [-π, π]
        """
        assert pose_ref.size == self.num_dofs()

        arm_pos = self.get_joint_state_arm()
        error = pose_ref - arm_pos 
        error_wrapped = (error + np.pi) % (2 * np.pi) - np.pi   # Normalize error to [-π, π] range
        assert np.all((-np.pi <= error_wrapped) & (error_wrapped <= np.pi)) 

        return error_wrapped
    
    def _check_alignment(self, pose_ref : np.ndarray) -> bool:
        """Checks if the robot aligns with a reference pose within some margin of error  
        
        :param print_error: whether to print an error table
        :param pose_ref: arm joint values to align to, in radians.
        :return: True if pose approximately matches; False otherwise
        """
        ERROR_MARGIN = np.pi/6
        assert 0 < ERROR_MARGIN < np.pi
        
        # Check arm joints
        error = self._check_wrapped_error(pose_ref)        
        error_abs = np.abs(error)
        within_bounds = bool(np.all(error_abs <= ERROR_MARGIN))
        arm_pos = self.get_joint_state_arm()
        
        # Move cursor up before printing
        if self._table_alr_printed:
            print(f"\033[{len(arm_pos) + 6}A", end="") 
        
        print("=" * 75) 
        print("REAL-TIME JOINT ERROR MONITORING")
        print("=" * 75)
        print(f"{'Joint':>6} | {'Target (°)':>10} | {'Actual (°)':>10} | {'Error (°)':>10} | {'Error (rad)':>12}")
        print("-" * 75)
        
        for i in range(len(arm_pos)):
            id = self._joint_ids[i]                
            target_deg = np.degrees(pose_ref[i])
            actual_deg = np.degrees(arm_pos[i])
            error_deg = np.rad2deg(error[i])
            error_rad = error[i]
            print(f"{id:6d} | {target_deg:10.1f} | {actual_deg:10.1f} | {error_deg:10.1f} | {error_rad:12.2f}")

        msg = (f"Close to reference position, keep arm stable" if within_bounds 
            else f"Arm deviation too far. Keep all joint errors below {np.rad2deg(ERROR_MARGIN):.1f}°")
        
        print(f"{msg}", end="\033[K\n") # \033[K ANSI code clears from cursor to end of line,

        self._table_alr_printed = True

        return within_bounds
        
    def calibrate(self, pose_ref: np.ndarray) -> bool:
        '''Runs a loop checking if arm joints are close to the reference pose provided. 
        If it is, calibration is done. A 30s timeout exists.

        :param pose_ref: joint values to align to, in radians. Gripper value, if provided, will be ignored.
        ''' 
        valid_sizes = (self.num_dofs(), self.num_dofs() + 1)
            
        assert pose_ref.size in valid_sizes, (f"Invalid pose size: {pose_ref.size}. "
                                              f"Expected size(s) of {valid_sizes}")
    
        if pose_ref.size != self.num_dofs():
            print("warning: wrong size provided")
            print(f"Using only the first {self.num_dofs()} joint values.")
            pose_ref = pose_ref[:-1]    

        TIMEOUT = 30.0
        STABLE_TIME = 1.0
        t_start = time.time()
        t_stable = None

        while True:
            if time.time() - t_start > TIMEOUT:
                print(f"Calibration time outed at {TIMEOUT} s")
                return False
            
            if not self._check_alignment(pose_ref):     # not aligned
                t_stable = None
                time.sleep(0.05)
                continue
            
            if t_stable is None:    # aligned
                t_stable = time.time()
            
            if time.time() - t_stable >= STABLE_TIME:
                break
            
            time.sleep(0.05)

        if self._set_joint_offsets(pose_ref) == True:
            print("Calibration successful")
            return True
        else:
            print("Calibration failed")
            return False        
    
    def _set_joint_offsets(self, pose_ref : np.ndarray) -> bool:
        """Finds amount of 2π wraps and sets offsets as it. Actual pose has to approximately match reference pose.

        :param pose: arm joint values to align to, in radians.
        :return: True if offsets were successfully determined and applied, False otherwise.
        """
        assert pose_ref.size == self.num_dofs(), \
            f"Expected {self.num_dofs()} reference positions for arm joints, got {pose_ref.size}"
        
        aligned = self._check_alignment(pose_ref) 

        if not aligned:
            print("Robot must be at reference pose before calling set_joint_offsets")
            self._is_calibrated = False
            return False
 
        arm_pos = self.get_joint_state_arm()
        error = pose_ref - arm_pos
        wrap_count = np.round(error / (2 * np.pi))
        offsets =  wrap_count * (2 * np.pi)
        self._joint_offsets = offsets
        
        self._is_calibrated = True

        return True
        
    def set_torque_mode(self, mode: bool):
        """Enables/disables controls."""
        if mode == self._torque_on:
            return
            
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Read the positions of all actuators, including that of gripper included (if any).

        Returns:
            Dictionary of joint positions in radians. If gripper is present,
            the last element is normalized to [0,1] where 0 = fully open,
            1 = fully closed.
        """
        return {"joint_state": self.get_joint_state()}

    def print_state(self):
        
        q = np.rad2deg(self.get_joint_state_arm())
        gripper = self.get_gripper_state()
        
        joint_str = "Joints: "
        for i, angle in enumerate(q):
            joint_str += f"J{i+1}: {angle:6.1f}°  "
        
        gripper_str = "Gripper: Not available" if gripper == None else\
              f"Gripper (0: open; 1: closed): {gripper:2.1f}"

        print(joint_str + " | " + gripper_str, end="\r")


    # FORCE CONTROL METHODS
    #TODO - create a kinematics class to take in
    def initialize_force(self, kinematics: ScaledURKinematics):
        self._kin_model = kinematics
        self._tau_max = self._get_servo_torque_limit()

    def _get_servo_torque_limit(self) -> np.ndarray:
        """Returns the max torque (in Nm) each arm servo can exert. Shape: (num_dof,)"""
        
        if isinstance(self._driver, FakeDynamixelDriver):
            raise RuntimeError("No torque limit for a fake driver")
        
        if self._driver.torque_limit is None:
            raise RuntimeError("Torque limits not configured. " \
            "Initialize DynamixelRobot with servo_types parameter.") 
        
        torque_limits = np.array(self._driver.torque_limit[:self.num_dofs()])
        assert torque_limits.shape == (self.num_dofs(), 1)

        return torque_limits

    
    #TODO - singularity 
    def set_wrench(self, wrench: np.ndarray):
        """Tries to set wrench"""

        # validate
        if self._kin_model is None:
            raise RuntimeError("Getting wrench limit requires kinematic model which is not initialized. " \
            "Initialize DynamixelRobot with kin_model parameter.")    

        if self._tau_max is None:
            raise RuntimeError("Max torque is not set, cannot find attentuation factor.")

        if len(wrench) != 6:
            raise ValueError("Invalid input. wrench should be of len 6.")
        
        #
        self.set_current_control_mode()
        out = self._get_attentuated_torque_for_wrench(wrench)
        factor, torque = out["factor"], out["torque"]     
        print()

        pass
    
    def _get_attentuated_torque_for_wrench(self, wrench: np.ndarray) -> dict: 
        """Gets a scaled down wrench that is producable by the robot and the scale factor"""
        
        # finds theoretical required torque
        q = self.get_joint_state_arm()
        assert self._kin_model is not None
        J = self._kin_model.get_jacobian_TCP(q)
        tau_req = J.T @ wrench
        assert self._tau_max is not None
        assert len(self._tau_max) == len(tau_req)

        # scale torque down for it to be produceable 
        with np.errstate(divide='ignore', invalid='ignore'):    # to suppress tau_max / 0 = inf and 0/0 errors
            scales = self._tau_max / np.abs(tau_req)
            scales = np.where(np.abs(tau_req) < 1e-8, np.inf, scales)   # so that unactiviated joints (0) wiil not limit scaling
        s = np.min(scales)      # s, is such that τ = Jᵀ (s * f_desired) satisfies |τ| ≤ τ_max where τ_max max torque of each servo.
        factor = min(float(s), 1.0) if np.isfinite(s) else 1.0
        tau_atten = tau_req * factor 

        # return dict
        return {
            "factor": factor,
            "torque": tau_atten  
        }

    def set_torque(self, torque: np.ndarray):
        """Sets the torque of arm actuators.`"""
        if len(torque) != self.num_dofs:
            raise ValueError("Torque provided is more than arm DOF.")
        
        torque_processed = list(torque * self._joint_signs)
        
        if self._has_gripper:
            torque_processed.append(0)  # gripper servo is idle

        if len(torque_processed) != self.num_actuators:
            raise RuntimeError("Unexpected")

        self._driver.set_torque(torque_processed)
    
    def set_current_control_mode(self):
        from gello.dynamixel.driver import CURRENT_CONTROL_MODE
        self._driver.set_torque_mode(False)          # must disable torque to change mode
        self._driver.set_operating_mode(CURRENT_CONTROL_MODE)
        self._driver.set_torque_mode(True)           # re-enable torque in current mode




@dataclass
class DynamixelRobotConfig:
    """A dataclass to store the configurations needed to initialize a Dynamixel robot.
    
    Attributes:
        joint_ids: actuator ids, e.g. (1, 2, 3...)
        joint_signs: joint direction multipliers, e.g. (1, -1, 1, ...)
        servo_types: model names, e.g. ("XL330_M288", ..., "XL330_M077")
        gripper_config: (Optional). Tuple of (id, open_degrees, closed_degrees). 
        If not provided, robot is initialized without a gripper
    """

    joint_ids: Sequence[int]
    joint_signs: Sequence[int]
    servo_types: Tuple[str, ...]
    gripper_config: Tuple[int, int, int] | None = None
    kinematics: ScaledURKinematics | None = None
    
    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_signs)     # same length
        assert all(0 <= id <= 252 for id in self.joint_ids)     # valid ids
        assert len(set(self.joint_ids)) == len(self.joint_ids)  # no duplicate ids
        assert np.all(np.abs(self.joint_signs) == 1)            # either 1 or -1        
        
        if self.gripper_config is None:
            return
        
        assert 0 < self.gripper_config[0] <= 252                # valid id
        assert self.gripper_config[0] not in self.joint_ids     # no duplicates
        assert -360 < self.gripper_config[1] < 360              # valid range
        assert -360 < self.gripper_config[2] < 360              # valid range
        
    def __str__(self) -> str:
        """Complete string representation with all configuration details."""

        info_lines = [
            f"DynamixelRobotConfig:",
            f"  Joints: {len(self.joint_ids)} servos",
            f"  IDs: {list(self.joint_ids)}",
            f"  Signs: {list(self.joint_signs)}",
        ]
        if self.servo_types == None:
            info_lines.extend([
                f"  Servo types: Not specified"
                ])
        else:
            info_lines.extend([
                f"  Servo types: {list(self.servo_types)}"
                ])
                    
        if self.gripper_config == None:
            info_lines.extend([
                f"  Gripper: None"
                ])
        else:
            id, open_pos, close_pos = self.gripper_config
            info_lines.extend([
                f"  Gripper:",
                f"    ID: {id}",
                f"    Open position: {open_pos}°",
                f"    Close position: {close_pos}°",
                f"    Range: {close_pos - open_pos}°"
                ])
            
        return "\n".join(info_lines)
    
    def make_robot(self, port: str = "/dev/ttyUSB0") -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=tuple(self.joint_ids),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            servo_types=self.servo_types,
        )
