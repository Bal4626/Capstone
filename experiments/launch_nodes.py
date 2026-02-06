from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot

@dataclass
class Args:
    robot: str = "ur"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.20.66"


def launch_robot_server(args: Args):
    port = args.robot_port  
    print(port)
    print(args.robot_ip)
    print("aaa")
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        # xml = MENAGERIE_ROOT / "universal_robots_ur3e" / "ur3e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "ur":
            print("zzz")
            from gello.robots.ur import URRobot
            print(args.robot_ip)
            robot = URRobot(robot_ip=args.robot_ip)
        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot

            # from gello.robots.dynamixel import DynamixelRobot
            

            # IP for the bimanual robot setup is hardcoded
            # WE MUST CHANGE THIS WHEN WE USING THE BIMANUAL ROBOT  -BALRAJ
            _robot_l = URRobot(robot_ip="192.168.20.66")
            _robot_r = URRobot(robot_ip="192.168.20.65")
            robot = BimanualRobot(_robot_l, _robot_r)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    print("xxx")
    launch_robot_server(args)


if __name__ == "__main__":

    print("yyy")
    main(tyro.cli(Args))
