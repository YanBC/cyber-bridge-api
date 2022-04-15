import argparse
import multiprocessing
import logging
from typing import Tuple
import xml.etree.ElementTree as ET

from utils import (
    load_json, load_tree, logging_wrapper)
from simulation import start_simulation


def get_args():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--carla-host',
        default='127.0.0.1',
        help='carla server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--carla-port',
        default=2000,
        type=int,
        help='carla port to connect to (default: 2000)')
    argparser.add_argument(
        '--apollo-host',
        default='127.0.0.1',
        help='apollo server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--apollo-port',
        default=9090,
        type=int,
        help='apollo port to connect to (default: 9090)')
    argparser.add_argument(
        '--dreamview-port',
        default=8888,
        type=int,
        help='apollo dreamview port (default: 8888)')
    argparser.add_argument(
        '--adc',
        default='hero',
        help='ego vehicle role name')
    argparser.add_argument(
        '--show',
        action='store_true',
        help='display adc in pygame')
    argparser.add_argument(
        '--timeout',
        type=float,
        default=20.0,
        help='Set the CARLA client timeout value in seconds')
    argparser.add_argument(
        '--sensor-config',
        default="sensor_configs/apollo_600_modular_testing.json",
        help='specify sensor config file path')
    argparser.add_argument(
        '--log-dir',
        default="./logs",
        help='where to store log files')
    argparser.add_argument(
        '--apollo-config',
        default='./apollo_configs/pnc_testing.json',
        help='path to apollo config file (default: apollo_configs/pnc_testing.json)')
    argparser.add_argument('configFile',
                           help='Scenario configuration file (*.json), '
                           'example: scenario_configs/run_1000km.json')
    argparser.add_argument("--fps",
                            type=int,
                            default=50,
                            help="Carla server update frequency, default to 50. "
                            "Set it to -1 if carla server should run in asynchronous mode. "
                            "This flag is incompatible with --show and takes precedence "
                            "when both are specified. Also note that this flag is "
                            "best-effort-only. The actual fps would depends on the hardware")
    args = argparser.parse_args()

    return args


@logging_wrapper
def main(args: argparse.Namespace):
    apollo_host = args.apollo_host
    apollo_port = args.apollo_port
    dreamview_port = args.dreamview_port
    carla_host = args.carla_host
    carla_port = args.carla_port
    ego_role_name = args.adc
    show = args.show
    carla_timeout = args.timeout
    log_dir = args.log_dir
    fps = args.fps
    sensor_config = load_json(args.sensor_config)
    apollo_config = load_json(args.apollo_config)
    sr_config = load_json(args.configFile)
    scenario_name = sr_config['scenario']
    scenario_config_tree = load_tree(sr_config['config'])
    sumo_cfg = sr_config.get("sumo", "")
    loop = sr_config.get("loop", False)

    # start simulation
    stop_event = multiprocessing.Event()
    logging.info(f"running scenario: {scenario_name}")
    result = start_simulation(
        stop_event=stop_event,
        apollo_host=apollo_host,
        apollo_port=apollo_port,
        dreamview_port=dreamview_port,
        carla_host=carla_host,
        carla_port=carla_port,
        scenario_name=scenario_name,
        scenario_config_tree=scenario_config_tree,
        sensor_config=sensor_config,
        apollo_config=apollo_config,
        fps=fps,
        log_dir=log_dir,
        ego_role_name=ego_role_name,
        carla_timeout=carla_timeout,
        show=show,
        sumo_cfg=sumo_cfg,
        loop=loop)
    logging.info(f"err_code: {result.err_code}")
    logging.info(f"criteria: {result.criteria}")


if __name__ == "__main__":
    args = get_args()
    log_dir = args.log_dir
    main(log_dir, 'main', args)
