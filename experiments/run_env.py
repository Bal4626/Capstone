import glob
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import tyro

from gello import env
from gello.agents import agent
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.utils.launch_utils import instantiate_from_dict
from gello.zmq_core.robot_node import ZMQClientRobot



def _match_dofs(vec, dof: int):
    """Return vec as np.ndarray of length == dof. If vec is longer by 1 (gripper), drop the last."""
    if vec is None:
        return None
    v = np.asarray(vec, dtype=float).reshape(-1)
    if v.size == dof + 1:
        v = v[:dof]          # drop gripper
    elif v.size > dof:
        v = v[:dof]          # be safe
    elif v.size < dof:
        v = np.pad(v, (0, dof - v.size))
    return v


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: Optional[str] = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False
    delta_mode: bool = True  # BALRAJ it automatically captures the delta at runtime

    def __post_init__(self):
        if self.start_joints is not None:
            self.start_joints = np.array(self.start_joints)


def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    agent_cfg = {}
    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0"
            # left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0"
            # right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0"
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": left,
                    "start_joints": (-1.57, -1.57, -1.57, -1.57,  1.57, 0.0),
                },
                "agent_right": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": right,
                    "start_joints":  ( 1.57, -1.57,  1.57, -1.57, -1.57, 0.0),
                },
            }
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "l",
                },
                "agent_right": {
                    "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "r",
                },
            }
        elif args.agent == "spacemouse":
            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": left_path,
                    "verbose": args.verbose,
                },
                "agent_right": {
                    "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": right_path,
                    "verbose": args.verbose,
                    "invert_button": True,
                },
            }
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([-90, -90, -90, -90, 90, 0])
        reset_joints_right = np.deg2rad([90, -90, 90, -90, -90, 0])
        # reset_joints_left = np.deg2rad([320, -100, -50, -150, 50, 50])
        # reset_joints_right = np.deg2rad([40, -70, 20, 20, -30, 340])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        print("curr_joints len:", len(env.get_obs()["joint_positions"]))
        print(f"Reset positions - Left: {np.rad2deg(reset_joints_left)}")
        print(f"Reset positions - Right: {np.rad2deg(reset_joints_right)}")
        
        # Handle shape mismatch if curr_joints includes gripper positions
        if len(curr_joints) != len(reset_joints):
            print(f"Shape mismatch: curr_joints={len(curr_joints)}, reset_joints={len(reset_joints)}")
            if len(curr_joints) > len(reset_joints):
                # Add gripper positions (0.0 for grippers)
                gripper_positions = np.zeros(len(curr_joints) - len(reset_joints))
                reset_joints = np.concatenate([reset_joints, gripper_positions])
                print(f"Added {len(gripper_positions)} gripper positions to reset_joints")

        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)
        print(f"Moving to reset positions with {steps} steps, max_delta: {np.rad2deg(max_delta):.1f} degrees")

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
            time.sleep(0.01)
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    gello_port = usb_ports[0] 
                    print(f"using port {gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )
            agent_cfg = {
                "_target_": "gello.agents.gello_agent.GelloAgent",
                "port": gello_port,
                "start_joints": args.start_joints,
            }
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    [-90, -90, -90, -90, 90, 0]
                )  # Change this to your own reset joints for the starting position
            else:
                reset_joints = np.array(args.start_joints)

            # curr_joints = env.get_obs()["joint_positions"]
            # curr_joints = np.array(curr_joints)

            curr_joints = np.asarray(env.get_obs()["joint_positions"], dtype=float).reshape(-1)
            reset_joints = _match_dofs(reset_joints, curr_joints.size)
            if reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                "robot_type": args.robot_type,
                "which_hand": "l",
            }
        elif args.agent == "spacemouse":
            agent_cfg = {
                "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                "robot_type": args.robot_type,
                "verbose": args.verbose,
            }
        elif args.agent == "dummy" or args.agent == "none":
            agent_cfg = {
                "_target_": "gello.agents.agent.DummyAgent",
                "num_dofs": robot_client.num_dofs(),
            }
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    agent = instantiate_from_dict(agent_cfg)


    ########################################################## BALRAJ

    # Delta-mode teleop: capture master & slave zero at runtime and command deltas.
    if args.delta_mode:
        obs0 = env.get_obs()
        slave_zero = np.asarray(obs0["joint_positions"], dtype=float).reshape(-1)
        master_zero = _match_dofs(agent.act(obs0), slave_zero.size)

        class DeltaBiasAgent:
            def __init__(self, base_agent, master_bias, slave_bias):
                self._base_agent = base_agent
                self._master_bias = np.asarray(master_bias, dtype=float).reshape(-1)
                self._slave_bias = np.asarray(slave_bias, dtype=float).reshape(-1)
                assert (
                    self._master_bias.size == self._slave_bias.size
                ), "Master/slave DOF mismatch"

            def act(self, obs):
                master_now = _match_dofs(
                    self._base_agent.act(obs), self._slave_bias.size
                )
                delta = master_now - self._master_bias
                return self._slave_bias + delta

        agent = DeltaBiasAgent(agent, master_zero, slave_zero)
        print("Delta teleop enabled: captured master/slave zeros and running in delta mode.")
    else:
        # Default absolute-start behaviour
        print("Going to start position")
        start_pos = agent.act(env.get_obs())
        obs = env.get_obs()
        joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)
        start_pos = _match_dofs(start_pos, joints.size)

        abs_deltas = np.abs(start_pos - joints)
        id_max_joint_delta = np.argmax(abs_deltas)

        max_joint_delta = 0.8
        if abs_deltas[id_max_joint_delta] > max_joint_delta:
            id_mask = abs_deltas > max_joint_delta
            print()
            ids = np.arange(len(id_mask))[id_mask]
            for i, delta, joint, current_j in zip(
                ids,
                abs_deltas[id_mask],
                start_pos[id_mask],
                joints[id_mask],
            ):
                print(
                    f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
                )
            return

        print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
        assert len(start_pos) == len(
            joints
        ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

        max_delta = 0.05
        for _ in range(25):
            obs = env.get_obs()
            current_joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)
            command_joints = _match_dofs(agent.act(obs), current_joints.size)

            delta = command_joints - current_joints
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > max_delta:
                delta = delta / max_joint_delta * max_delta
            env.step(current_joints + delta)

        obs = env.get_obs()
        joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)
        action = _match_dofs(agent.act(obs), joints.size)

        diff = action - joints
        bad = np.where(diff > 0.8)[0]
        for j in bad:
            print(
                f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
            )
            exit()
        if (action - joints > 0.8).any():
            print("Action is too big")

            # print which joints are too big
            joint_index = np.where(action - joints > 0.8)
            for j in joint_index:
                print(
                    f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                )
            exit()

    from gello.utils.control_utils import SaveInterface, run_control_loop

    save_interface = None
    if args.use_save_interface:
        save_interface = SaveInterface(
            data_dir=args.data_dir, agent_name=args.agent, expand_user=True
        )

    run_control_loop(env, agent, save_interface, use_colors=True)


if __name__ == "__main__":
    main(tyro.cli(Args))
