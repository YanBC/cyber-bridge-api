import argparse
import multiprocessing

from utils import load_json, load_tree
from simulation import startup_simulation


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
    argparser.add_argument('--configFile',
                           default='./scenario_configs/'
                           'free_ride.json',
                           help='Provide a scenario configuration file (*.json), '
                           'default: scenario_configs/free_ride.json')
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


def main(args: argparse.Namespace):
    # logging.basicConfig(level=logging.INFO)
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

    # start simulation
    stop_event = multiprocessing.Event()
    result = startup_simulation(
        log_dir,
        "simulation",
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
        child_proc_log_dir=log_dir,
        ego_role_name=ego_role_name,
        carla_timeout=carla_timeout,
        show=show)
    print(result.err_code)
    print(result.criteria)


if __name__ == "__main__":
    args = get_args()
    main(args)
