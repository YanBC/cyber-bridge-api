import argparse
import random
import time

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import carla
except ImportError:
    raise RuntimeError('cannot import carla, make sure carla package is installed')

# PYTHONPATH should be set
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.basic_agent import BasicAgent
from encoders.apollo_control_encoder import ApolloControlEncoder
from modules.control.proto.control_cmd_pb2 import ControlCommand
from cyber_bridge_client import CyberBridgeClient


def get_args():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '--carla',
        default='127.0.0.1',
        help='carla server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--carla-port',
        default=2000,
        type=int,
        help='carla port to connect to (default: 2000)')
    argparser.add_argument(
        '--apollo',
        default='127.0.0.1',
        help='apollo server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--apollo-port',
        default=9090,
        type=int,
        help='apollo port to connect to (default: 9090)')
    argparser.add_argument(
        '--adc',
        default='hero',
        help='ego vehicle name')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument(
        '--channel',
        default="/apollo/control",
        help="Apollo control channel name"
    )
    argparser.add_argument(
        '--msgtype',
        default="apollo.control.ControlCommand",
        help="Apollo control channel message type"
    )
    argparser.add_argument(
        '--debug',
        default=False,
        type=bool,
        help='whether to display routing waypoint')

    args = argparser.parse_args()
    return args


def main():
    args = get_args()

    if args.seed:
        random.seed(args.seed)

    client = carla.Client(args.carla, args.carla_port)
    client.set_timeout(4.0)

    sim_world = client.get_world()
    player = None
    player_name = None

    while player is None:
        print("Waiting for the ego vehicle...")
        time.sleep(1)
        possible_vehicles = sim_world.get_actors().filter('vehicle.*')
        for vehicle in possible_vehicles:
            if vehicle.attributes['role_name'] == args.adc:
                print("Ego vehicle found")
                player = vehicle
                player_name = player.type_id
                break

    if args.agent == "Basic":
        agent = BasicAgent(player)
    else:
        agent = BehaviorAgent(player, behavior=args.behavior)

    destination = carla.Location(x=-50, y=-195)
    initialposi = player.get_location()
    agent.set_destination(initialposi, destination)

    apollo_host = args.apollo
    apollo_port = args.apollo_port
    apollo_channel = args.channel
    apollo_msgType = args.msgtype

    ctrl_encoder = ApolloControlEncoder(
            ControlCommand,
            apollo_channel,
            apollo_msgType)
    cb_client = CyberBridgeClient(
            apollo_host, apollo_port,
            [ctrl_encoder], [])
    cb_client.initialize()

    while True:
        sim_world.wait_for_tick(seconds=3.0)
        agent.update_information()

        if len(sim_world.get_actors().filter(player_name)) < 1:
            print("ego vehicle destroyed, stopping...")
            break

        if len(agent.get_local_planner().waypoints_queue) == 0:
            print("Target reached, stopping...")
            break

        speed_limit = player.get_speed_limit()
        agent.get_local_planner().set_speed(speed_limit)

        control = agent.run_step(debug=args.debug)

        pbCtrl = ctrl_encoder.carlaToProtobuf(control)
        send_list = [ctrl_encoder.get_publish_bytes(pbCtrl)]
        cb_client.send_pb_messages(send_list)


if __name__ == '__main__':
    main()
