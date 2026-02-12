"""
This class handles the mid level implementation of the Dynamixel robot.

Lower level implementation is in the DynamixelDriver class.
"""

from typing import Dict, Optional, Sequence, Tuple
import numpy as np
import time

from dataclasses import dataclass
from gello.robots.robot import Robot
from gello.dynamixel.driver import DynamixelDriver, FakeDynamixelDriver, CURRENT_CONTROL_MODE, POSITION_CONTROL_MODE
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
        """Initializes a Dynamixel robot. 
        The gripper is only initialized when gripper_config is provided.   
        
        :param joint_ids: List of Dynamixel motor IDs for the arm joints
        :param joint_signs: Direction multipliers (1 or -1) for each arm joint
        :param servo_types: List of servo models at arm joints 
        :param real: Whether to use real hardware or simulation
        :param port: Serial port for real hardware
        :param baudrate: Communication baud rate
        :param gripper_config: (Optional) tuple of (gripper_id, open_angle, close_angle) in degrees. If not specified, will be initialized without a gripper
        :param servo_types: (Optional)[Sequence[str]] = None
        """
        # validate inputs
        if len(joint_signs) != len(joint_ids):
            raise ValueError("joint_signs and joint_ids length must match")
        if len(joint_signs) != len(servo_types):
            raise ValueError("joint_signs and servo_types length must match")
        if not np.all(np.abs(joint_signs) == 1):
            raise ValueError("joint_signs must be 1 or -1")
        
        # initialize fields 
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
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99
        self._table_printed = False
        self._is_calibrated = False

        # check for driver and input consistency 
        if len(self._driver.get_joints()) != self.num_actuators():
            raise RuntimeError(f"Driver found {len(self._driver.get_joints())} actuators but expected {self.num_actuators()}")
        
        # set to current mode 
        self.set_current_control_mode()

        # allow freedrive
        self._driver.set_torque_mode(False)

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

    def _get_joint_state(self) -> np.ndarray:
        """Returns the positions of all actuators. 
        If gripper is present, the last element is normalized to [0,1] where 0=fully open, 1=fully closed.
        """
        # require calibration
        if self._is_calibrated == False:
            raise RuntimeError("Arm is not calibrated. Calibration required before values can be returned.")
            # raise RuntimeWarning("Arm is not calibrated. Raw unprocessed values are being returned")

        # process raw joint values
        readings = self._driver.get_joints()
        arm_raw = readings[:self.num_dofs()]
        assert arm_raw.size == self.num_dofs()
        arm_calibrated = arm_raw * self._joint_signs + self._joint_offsets

        #  process raw gripper values (if present)
        gripper_processed = None
        if self._has_gripper:
            gripper_raw = readings[-1]
            assert self._gripper_open_close is not None
            open_angle, close_angle = self._gripper_open_close
            gripper_norm = (gripper_raw - open_angle) / (close_angle - open_angle)
            gripper_processed = np.clip(gripper_norm, 0, 1)

        # combine and return results
        positions = arm_calibrated if gripper_processed is None \
            else np.append(arm_calibrated, gripper_processed) 
        assert positions.size == self.num_actuators()
        return self._smooth(positions)

    def get_joint_state(self) -> np.ndarray:
        """Returns the positions of arm joints in radians."""
        return  self._get_joint_state()[:self.num_dofs()]
    
    def get_gripper_state(self) -> float | None:
        """Returns the normalized gripper position, if present. Otherwise None"""
        return self._get_joint_state()[-1] if self._has_gripper else None

    def command_joint_state(self, joint_state: np.ndarray):
        """Send position commands to arm joints. 
        
        :param joint_state: Target positions in radians
        # TODO test
        """
        # validate input
        if len(joint_state) != self.num_dofs():
            raise ValueError(f"Expected {self.num_dofs()} joints values, got {len(joint_state)}")
        
        # allow movement only when calibrated 
        if self._is_calibrated == False:
            raise RuntimeError("Arm not calibrated. Calibration required before moving")

        # convert joint values to raw values 
        joint_raw = joint_state / self._joint_signs - self._joint_offsets

        #  append gripper current value (if it exists)
        if self._has_gripper:
            gripper_raw = self._driver.get_joints()[-1]
            all_raw = np.append(joint_raw, gripper_raw)
        else:
            all_raw = joint_raw

        # set actuators
        self._driver.set_joints(all_raw.tolist())

    def command_gripper_state(self, state: float):
        """Send position command to gripper.
        
        :param state: Normalized gripper value in [0,1], where 0=open, 1=closed
        # TODO test
        """
        # validate input
        if not self._has_gripper:
            raise RuntimeError("Called command_gripper_state method when there is no gripper")
        if not (0 <= state <= 1):
            raise ValueError(f"Normalized gripper value must be in [0,1], got {state}")

        # convert normalized value to raw value
        open_angle, close_angle = self._gripper_open_close
        gripper_raw = state * (close_angle - open_angle) + open_angle

        # set actuators
        joint_raw = self._driver.get_joints()[:self.num_dofs()]
        all_raw : np.ndarray = np.append(joint_raw, gripper_raw)
        self._driver.set_joints(all_raw.tolist())
        
    def _get_wrapped_error(self, pose_ref : np.ndarray) -> np.ndarray:
        """Returns the wrapped error between current arm pose and reference pose, in radians."""
        # get raw error
        assert pose_ref.size == self.num_dofs() # pose_ref is desired positions (rad)
        arm_pos = self.get_joint_state()
        error = pose_ref - arm_pos

        # return wrapped error
        error_wrapped = (error + np.pi) % (2 * np.pi) - np.pi   # wrapped error is in [-π, π] range
        assert np.all((-np.pi <= error_wrapped) & (error_wrapped <= np.pi)) 
        return error_wrapped
    
    def _get_alignment(self, pose_ref : np.ndarray) -> bool:
        """Checks if arm joints are aligned with a reference pose within some margin of error. 
        Returns True if pose approximately matches; False otherwise
        """
        # set margin
        ERROR_MARGIN = np.pi/6
        assert 0 < ERROR_MARGIN < np.pi
        
        # check if joints within margin
        error = self._get_wrapped_error(pose_ref)        
        error_abs = np.abs(error)
        within_bounds = bool(np.all(error_abs <= ERROR_MARGIN))
        arm_pos = self.get_joint_state()
        
        # move cursor up for table reprint
        if self._table_printed:
            print(f"\033[{len(arm_pos) + 6}A", end="") 
        
        # print table
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
        
        # print feedback line
        msg = (f"Close to reference position, keep arm stable" if within_bounds 
            else f"Arm deviation too far. Keep all joint errors below {np.rad2deg(ERROR_MARGIN):.1f}°")
        print(f"{msg}", end="\033[K\n") # \033[K ANSI code clears from cursor to end of line,

        # note table print
        self._table_printed = True

        # return align status
        return within_bounds
        
    def calibrate(self, pose_ref: np.ndarray) -> bool:
        '''Runs a loop checking if arm joints are close to the reference pose provided. 
        If it is, calibration is done. A 30s timeout exists.

        :param pose_ref: joint values to align to, in radians.

        Returns True if calubration succesfull; False otherwise
        ''' 
        # validate input
        if pose_ref.size != self.num_dofs():            
            raise ValueError(f"Invalid pose size: {pose_ref.size}. Expected size of {self.num_dofs()}")

        # set timer settings
        TIMEOUT = 30.0
        STABLE_TIME = 1.0   # time to wait while stable before exit 
        t_start = time.time()
        t_stable = None

        # run timer loop
        while True:
            if time.time() - t_start > TIMEOUT:
                print(f"Calibration time outed at {TIMEOUT} s")
                return False
            # not aligned
            if not self._get_alignment(pose_ref):
                t_stable = None
                time.sleep(0.05)
                continue
            # aligned
            if t_stable is None:
                t_stable = time.time()
            if time.time() - t_stable >= STABLE_TIME:
                break
            time.sleep(0.05)

        # calibrate on stable
        calibrated = self._set_joint_offsets(pose_ref)        
        if not calibrated:
            print("Calibration failed")

        # return outcome
        return calibrated     
    
    def _set_joint_offsets(self, pose_ref : np.ndarray) -> bool:
        """Finds amount of 2π wraps and sets offsets as it. Actual pose has to approximately match reference pose. 
        Returns True if offsets were successfully determined and applied, False otherwise.
        """
        # ensure aligned before proceeding
        assert pose_ref.size == self.num_dofs()        
        if self._get_alignment(pose_ref) == False:
            print("Robot must be at reference pose before calling set_joint_offsets")
            self._is_calibrated = False
            return False

        # set joint offset
        arm_pos = self.get_joint_state()
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

    def get_observations(self) -> Dict:
        """Read the positions of all actuators, including normalized gripper value (if any).

        Returns:
            Dictionary of joint positions in radians. If gripper is present,
            the last element is normalized to [0,1] where 0 = fully open,
            1 = fully closed.
        """
        return {"all_state": self.get_joint_state(),
                "joint_state": self.get_joint_state(),
                "gripper_state": self.get_gripper_state(),}

    def print_status(self):
        """Prints robot status"""
        q = np.rad2deg(self.get_joint_state())
        gripper = self.get_gripper_state()
        joint_str = "Joints: "
        for i, angle in enumerate(q):
            joint_str += f"J{i+1}: {angle:6.1f}°  "
        gripper_str = "Gripper: Not available" if gripper == None else\
              f"Gripper (0: open; 1: closed): {gripper:2.1f}"
        print(joint_str + " | " + gripper_str, end="\r")


    # FORCE CONTROL METHODS
    def initialize_force(self, kinematics: ScaledURKinematics): #TODO - create a kinematics class to take in
        """Initialize this to use force control methods. """
        self._kin_model = kinematics
        self._tau_max = self._get_servo_torque_limit()

    def _get_servo_torque_limit(self) -> np.ndarray:
        """Returns the max torque (in Nm) each arm servo can exert. Shape: (num_dof,)"""
        # check prerequisites
        if isinstance(self._driver, FakeDynamixelDriver):
            raise RuntimeError("No torque limit for a fake driver")
        if self._driver.torque_limit is None:
            raise RuntimeError("Torque limits not configured. " \
            "Initialize DynamixelRobot with servo_types parameter.") 
        
        # return limits
        torque_limits = np.array(self._driver.torque_limit[:self.num_dofs()])
        assert torque_limits.shape == (self.num_dofs(), 1)
        return torque_limits

    
    #TODO - handle singularity situations 
    def set_wrench(self, wrench: np.ndarray):
        """Sets the joint torque necessary to produce a given wrench at the TCP.
        If the wrench is too large, it is scaled down to a producable value.

        :param wrench: [fx, fy, fz, mx, my, mz] (N and Nm) expressed in base frame.
        """

        # validate inputs
        if self._kin_model is None:
            raise RuntimeError("Getting wrench limit requires kinematic model which is not initialized. " \
            "Initialize DynamixelRobot with kin_model parameter.")    
        if self._tau_max is None:
            raise RuntimeError("Max torque is not set, cannot find attentuation factor.")
        if len(wrench) != 6:
            raise ValueError("Invalid input. wrench should be of len 6.")

        # set torque
        self._driver.verify_operating_mode(CURRENT_CONTROL_MODE)
        out = self._get_attentuated_torque_for_wrench_at_TCP(wrench)
        factor, torque = out["factor"], out["torque"]     
        self.set_torque(torque)
        
        # print actual wrench
        t = wrench  # target
        a = wrench * factor     # actual
        print(f"Target: [{t[0]:5.1f} {t[1]:5.1f} {t[2]:5.1f} | {t[3]:5.1f} {t[4]:5.1f} {t[5]:5.1f}]  "
            f"Factor: {factor:.2f}  "
            f"Actual: [{a[0]:5.1f} {a[1]:5.1f} {a[2]:5.1f} | {a[3]:5.1f} {a[4]:5.1f} {a[5]:5.1f}]")
    
    def _get_attentuated_torque_for_wrench_at_TCP(self, wrench: np.ndarray) -> dict: 
        """Gets a scaled down wrench that is producable by the robot and the scale factor"""
        
        # finds theoretical required torque
        q = self.get_joint_state()
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
        """Sets the torque of arm actuators."""
        # validate input
        if len(torque) != self.num_dofs:
            raise ValueError("Torque provided is more than arm DOF.")
        
        # set torque
        torque_processed = list(torque * self._joint_signs) # re
        if self._has_gripper:
            torque_processed.append(0)  # gripper servo is idle
        assert len(torque_processed) == self.num_actuators
        self._driver.set_torque(torque_processed)
    
    def set_current_control_mode(self):
        self._driver.set_torque_mode(False)          # must disable torque to change mode
        self._driver.set_operating_mode(CURRENT_CONTROL_MODE)
        self._driver.set_torque_mode(True)           # re-enable torque in current mode

    def set_position_control_mode(self):
        self._driver.set_torque_mode(False)
        self._driver.set_operating_mode(POSITION_CONTROL_MODE)
        self._driver.set_torque_mode(True)

    




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
