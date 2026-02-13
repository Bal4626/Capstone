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



# def _match_dofs(vec, dof: int):
#     """Return vec as np.ndarray of length == dof. If vec is longer by 1 (gripper), drop the last."""
#     if vec is None:
#         return None
#     v = np.asarray(vec, dtype=float).reshape(-1)
#     if v.size == dof + 1:
#         v = v[:dof]          # drop gripper
#     elif v.size > dof:
#         v = v[:dof]          # be safe
#     elif v.size < dof:
#         v = np.pad(v, (0, dof - v.size))
#     return v


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
    robot_type: Optional[str] = None   # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[np.ndarray] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False

    # Inside your existing @dataclass Args:
    simulate: bool = False
    sim_force: Tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # Fx, Fy, Fz, Mx, My, Mz
    force_control: bool = False

    def __post_init__(self):
        if self.start_joints is not None:
            self.start_joints = np.array(self.start_joints)


def main(args):
    if args.simulate:
        from gello.robots.simulated_ur5 import SimulatedUR5
        print_color("🚀 Running in SIMULATION mode", color="yellow")
        robot_client = SimulatedUR5(
            start_joints=args.start_joints,
            constant_force=np.array(args.sim_force),
            use_gripper=False  # or True if you want 7-DOF
        )
        camera_clients = {}

    elif args.mock:
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
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6Z5LY0-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NN69-if00-port0"
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": left,
                    "start_joints": (1.57, -1.57, 1.57, -1.57, -1.57, 0, 0)
                },
                "agent_right": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": right,
                    "start_joints":(-1.57, -1.57, -1.57, -1.57, 1.57, 0, 0)
                },
            }

        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([90, -90, 90, -90, -90, 0, 0])
        reset_joints_right = np.deg2rad([-90, -90, -90, -90, 90, 0, 0])

        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
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
                    [90, -90, 90, -90, -90, 0, 0]
                )  # Change this to your own reset joints for the starting position
                
            else:
                reset_joints = np.array(args.start_joints)

            curr_joints = np.asarray(env.get_obs()["joint_positions"])
            # reset_joints = _match_dofs(reset_joints, curr_joints.size)
            if reset_joints is not None and reset_joints.shape == curr_joints.shape:
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
    # going to start position
    print("Going to start position")
    start_pos = agent.act(env.get_obs())
    obs = env.get_obs()
    joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)  # (6,)
    # start_pos = _match_dofs(start_pos, joints.size)
    print("ur5 joints",joints)
    print("dynamixel joints:",start_pos)
    print("differences:", start_pos - joints)
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
        # command_joints = agent.act(obs)
        # current_joints = obs["joint_positions"]

        current_joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)
        command_joints = agent.act(obs)

        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)

    obs = env.get_obs()
    # joints = obs["joint_positions"]
    # action = agent.act(obs)

    joints = np.asarray(obs["joint_positions"], dtype=float).reshape(-1)
    action = agent.act(obs)
    if (action - joints > 0.5).any():
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

    run_control_loop(env, agent, save_interface, use_colors=True, force_control=args.force_control)

if __name__ == "__main__":
    main(tyro.cli(Args))
