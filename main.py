import argparse
import time
import multiprocessing
import os
import functools
import logging

import carla
from sensors.apollo_control import listen_and_apply_control
from sensors.utils import setup_sensors
from pygame_viewer import view_game
from utils import is_actor_exist, load_json, get_vehicle_by_role_name
from scenario_runner import scenario_run
from scenario_parser import ScenarioConfigurationParser as SrCfgP
from dreamview_api import Connection as DreamviewConn


def destroy_all_sensors(world):
    sensor_list = world.get_actors().filter("*sensor*")
    """Destroys all actors"""
    for actor in sensor_list:
        if actor is not None:
            actor.destroy()


def logging_wrapper(func):
    @functools.wraps(func)
    def wrapper(log_dir:str, name:str, *args, **kwargs):
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        timestr = time.strftime("%Y%m%d-%H%M%S")
        logfilepath = os.path.join(log_dir, f"{name}.{timestr}.log")
        if os.path.isfile(logfilepath):
            os.remove(logfilepath)
        logging.basicConfig(
                filename=logfilepath,
                level=logging.INFO,
                format='%(asctime)s %(message)s')
        func(*args, **kwargs)
    return wrapper


@logging_wrapper
def run_pygame(*args, **kwargs):
    return view_game(*args, **kwargs)


@logging_wrapper
def run_control(*args, **kwargs):
    return listen_and_apply_control(*args, **kwargs)


@logging_wrapper
def run_sensors(*args, **kwargs):
    return setup_sensors(*args, **kwargs)


@logging_wrapper
def run_scenario(*args, **kwargs):
    return scenario_run(*args, **kwargs)


def get_args():
    argparser = argparse.ArgumentParser(
        description='Run scenarios which run in carla, '
        'and the AD statck is Apollo')
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
    argparser.add_argument('--configFile',
                           default='./scenario_configs/'
                           '743_borrow_lane_cfg.json',
                           help='Provide a scenario configuration file '
                           '(*.json)')
    args = argparser.parse_args()

    # communication with apollo and carla args
    ac_args = argparse.Namespace()
    ac_args.carla_host = args.carla_host
    ac_args.carla_port = args.carla_port
    ac_args.apollo_host = args.apollo_host
    ac_args.apollo_port = args.apollo_port
    ac_args.dreamview_port = args.dreamview_port
    ac_args.adc = args.adc
    ac_args.show = args.show
    ac_args.timeout = args.timeout
    ac_args.sensor_config = args.sensor_config
    ac_args.log_dir = args.log_dir

    # sr_host_keys = ['host', 'port', 'timeout']
    sr_host_dict = dict()
    sr_host_dict.update({'host': args.carla_host})
    sr_host_dict.update({'port': args.carla_port})
    sr_host_dict.update({'timeout': args.timeout})

    scenario_configurations = []
    sr_config = load_json(args.configFile)
    sr_config.update({'list': False})
    scenario_configurations = \
        SrCfgP.parse_scenario_configuration_with_customer_param(
                sr_config['scenario'],
                sr_config['configFile'])
    sr_config.update(
        {'scenario_configurations': scenario_configurations})

    if len(scenario_configurations) > 0:
        ac_args.dst_x = scenario_configurations[0].destination.x
        ac_args.dst_y = scenario_configurations[0].destination.y
        ac_args.dst_z = scenario_configurations[0].destination.z

    # scenario runner args
    sr_args = {**sr_host_dict, **sr_config}
    return ac_args, argparse.Namespace(**sr_args)


def main(ac_args: argparse.Namespace, sr_args: argparse.Namespace):
    # logging.basicConfig(level=logging.INFO)
    apollo_host = ac_args.apollo_host
    apollo_port = ac_args.apollo_port
    dreamview_port = ac_args.dreamview_port
    carla_host = ac_args.carla_host
    carla_port = ac_args.carla_port
    ego_role_name = ac_args.adc
    show = ac_args.show
    carla_timeout = ac_args.timeout
    sensor_config = load_json(ac_args.sensor_config)
    log_dir = ac_args.log_dir
    dst_x = ac_args.dst_x
    dst_y = ac_args.dst_y
    dst_z = ac_args.dst_z

    sim_world = None
    child_pid_file = open("pids.txt", "w")
    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(carla_timeout)

        sim_world = client.get_world()
        sim_map = sim_world.get_map()
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.02
        sim_world.apply_settings(settings)

        scenario_runner = multiprocessing.Process(
                        target=run_scenario,
                        args=(log_dir, "scenario_runner", sr_args))
        scenario_runner.start()
        child_pid_file.write(f"scenario_runner pid: {scenario_runner.pid}\n")

        player, _ = get_vehicle_by_role_name(
                __name__, sim_world, ego_role_name)
        dreamview_conn = DreamviewConn(apollo_host, dreamview_port)
        start_waypoint = sim_map.get_waypoint(player.get_location())
        end_waypoint = sim_map.get_waypoint(
                carla.Location(x=dst_x, y=dst_y, z=dst_z))
        dreamview_conn.set_destination(start_waypoint, end_waypoint)

        if show:
            viewer = multiprocessing.Process(
                    target=run_pygame,
                    args=(log_dir, "view_game", ego_role_name, carla_host,
                          carla_port, 1080, 720))
            viewer.start()
            child_pid_file.write(f"viewer pid: {viewer.pid}\n")

        control_sensor = multiprocessing.Process(
                            target=run_control,
                            args=(log_dir, "apollo_control", ego_role_name,
                                  carla_host, carla_port, apollo_host,
                                  apollo_port))
        control_sensor.start()
        child_pid_file.write(f"control_sensor pid: {control_sensor.pid}\n")
        sensors_config = multiprocessing.Process(
                            target=run_sensors,
                            args=(log_dir, "carla_sensors",
                                  ego_role_name, carla_host,
                                  carla_port, apollo_host, apollo_port,
                                  sensor_config))
        sensors_config.start()
        child_pid_file.write(f"sensors_config pid: {sensors_config.pid}\n")

        child_pid_file.close()
        while True:
            if not is_actor_exist(sim_world, role_name=ego_role_name):
                break
            time.sleep(1)

    finally:
        if not child_pid_file.closed:
            child_pid_file.close()
        if sim_world is not None:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            sim_world.apply_settings(settings)
            destroy_all_sensors(sim_world)


if __name__ == "__main__":
    ac_args, sr_args = get_args()
    main(ac_args, sr_args)
