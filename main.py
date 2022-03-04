import argparse
from telnetlib import EC
import time
import multiprocessing
import os
import functools
import logging
import pygame
import math

import carla
from sensors.apollo_control import listen_and_apply_control
from sensors.utils import setup_sensors
from pygame_viewer import view_game
from utils import acquire_servers, is_actor_exist, load_json, get_vehicle_by_role_name
from scenario_runner.scenario_runner import scenario_run
from srunner.tools.scenario_parser \
    import ScenarioConfigurationParser as SrCfgP
from dreamview_api import setup_apollo, reset_apollo


def destroy_all_sensors(world):
    sensor_list = world.get_actors().filter("*sensor*")
    """Destroys all actors"""
    for actor in sensor_list:
        if actor is not None:
            actor.destroy()


def logging_wrapper(func):
    @functools.wraps(func)
    def wrapper(log_dir: str, name: str, *args, **kwargs):
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
        "--discover",
        default="",
        help="service dicovery addr (ip:port)"
        "if specified, flags about apollo and carla server "
        "ip addr and port like --carla-host, --apollo-host will be"
        "ignored")
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
                            "best-effort-only. The actual fps would be depends on the hardware")
    args = argparser.parse_args()

    # communication with apollo and carla args
    ac_args = argparse.Namespace()
    stop_event = multiprocessing.Event()
    if args.discover != "":
        # TODO
        # duration is hardcoded to 5 mins because there is
        # no way to tell how long a simulation will run
        # for now.
        apollo_host, carla_host = acquire_servers(
            stop_event, args.discover, duration=60*5)
        args.carla_host = carla_host
        args.carla_port = 2000
        args.apollo_host = apollo_host
        args.apollo_port = 9090
        args.dreamview_port = 8888
        print(f"Dreamview at {args.apollo_host}:{args.dreamview_port}")
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
    apollo_config = load_json(args.apollo_config)
    ac_args.dreamview_mode = apollo_config['mode']
    ac_args.apollo_modules = apollo_config['modules']
    ac_args.fps = args.fps

    # sr_host_keys = ['host', 'port', 'timeout']
    sr_host_dict = dict()
    sr_host_dict.update({'host': args.carla_host})
    sr_host_dict.update({'port': args.carla_port})
    sr_host_dict.update({'timeout': args.timeout})

    scenario_configurations = []
    sr_config = load_json(args.configFile)
    sr_config.update({'sync': False})
    sr_config.update({'list': False})
    scenario_configurations = \
        SrCfgP.parse_scenario_configuration_with_customer_param(
                sr_config['scenario'],
                sr_config['configFile'])
    sr_config.update(
        {'scenario_configurations': scenario_configurations})

    if len(scenario_configurations) > 0:
        try:
            ac_args.dst_x = scenario_configurations[0].destination.x
            ac_args.dst_y = scenario_configurations[0].destination.y
            ac_args.dst_z = scenario_configurations[0].destination.z
        except Exception:
            ac_args.dst_x = ac_args.dst_y = ac_args.dst_z = None
        ego = scenario_configurations[0].ego_vehicles[0]
        ac_args.srt_x = ego.transform.location.x
        ac_args.srt_y = ego.transform.location.y
        ac_args.srt_z = ego.transform.location.z

    # scenario runner args
    sr_args = {**sr_host_dict, **sr_config}
    return ac_args, argparse.Namespace(**sr_args), stop_event


def get_args_external(arguments: argparse.Namespace()):
    args = arguments

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
    apollo_config = load_json(args.apollo_config)
    ac_args.dreamview_mode = apollo_config['mode']
    ac_args.apollo_modules = apollo_config['modules']
    ac_args.fps = args.fps

    # sr_host_keys = ['host', 'port', 'timeout']
    sr_host_dict = dict()
    sr_host_dict.update({'host': args.carla_host})
    sr_host_dict.update({'port': args.carla_port})
    sr_host_dict.update({'timeout': args.timeout})

    scenario_configurations = []
    sr_config = load_json(args.configFile)
    sr_config.update({'sync': False})
    sr_config.update({'list': False})
    sr_config.update({'json': True})
    scenario_configurations = \
        SrCfgP.parse_scenario_configuration_with_customer_param(
                sr_config['scenario'],
                sr_config['configFile'])
    sr_config.update(
        {'scenario_configurations': scenario_configurations})

    if len(scenario_configurations) > 0:
        try:
            ac_args.dst_x = scenario_configurations[0].destination.x
            ac_args.dst_y = scenario_configurations[0].destination.y
            ac_args.dst_z = scenario_configurations[0].destination.z
        except Exception:
            ac_args.dst_x = ac_args.dst_y = ac_args.dst_z = None
        ego = scenario_configurations[0].ego_vehicles[0]
        ac_args.srt_x = ego.transform.location.x
        ac_args.srt_y = ego.transform.location.y
        ac_args.srt_z = ego.transform.location.z

    # scenario runner args
    sr_args = {**sr_host_dict, **sr_config}
    return ac_args, argparse.Namespace(**sr_args)


def player_is_ready(vehicle: carla.Vehicle):
    acce = vehicle.get_acceleration()
    a = math.sqrt(
            math.pow(acce.x, 2) + math.pow(acce.y, 2) +
            math.pow(acce.z, 2))
    if a > 0:
        return False
    else:
        return True


def wait_vehicle_stable(world, vehicle):
    if world.get_settings().synchronous_mode:
        world.tick()
    while(True):
        if player_is_ready(vehicle):
            break
        if world.get_settings().synchronous_mode:
            world.tick()


def main(
        ac_args: argparse.Namespace,
        sr_args: argparse.Namespace,
        stop_event: multiprocessing.Event):
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
    srt_x = ac_args.srt_x
    srt_y = ac_args.srt_y
    srt_z = ac_args.srt_z
    dreamview_mode = ac_args.dreamview_mode
    apollo_modules = ac_args.apollo_modules
    kv_map_names = load_json("./kv_mappings/maps.json")
    kv_vehicle_names = load_json("./kv_mappings/vehicles.json")
    carla_map = sr_args.scenario_configurations[0].town
    carla_vehicle = sr_args.scenario_configurations[0].ego_vehicles[0].model
    apollo_map = kv_map_names.get(carla_map, None)
    if apollo_map is None:
        logging.error(f"No Apollo map for {carla_map}")
        return
    apollo_vehicle = kv_vehicle_names.get(carla_vehicle, None)
    if apollo_vehicle is None:
        logging.error(f"No Apollo vehicle for {apollo_vehicle}")
        return
    fps = ac_args.fps

    sim_world = None
    child_pid_file = open("pids.txt", "w")
    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(carla_timeout)

        sim_world = client.get_world()
        sim_map = sim_world.get_map()
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02
        settings.no_rendering_mode = True
        sim_world.apply_settings(settings)

        start_waypoint = sim_map.get_waypoint(
            carla.Location(x=srt_x, y=srt_y, z=srt_z))
        if dst_x is not None:
            end_waypoint = sim_map.get_waypoint(
                carla.Location(x=dst_x, y=dst_y, z=dst_z))
        else:
            end_waypoint = None
        if not setup_apollo(
                apollo_host,
                dreamview_port,
                dreamview_mode,
                apollo_map,
                apollo_vehicle,
                apollo_modules,
                start_waypoint,
                end_waypoint):
            logging.error("Apollo setup fail. Exiting ...")
            return

        scenario_runner = multiprocessing.Process(
                        target=run_scenario,
                        args=(log_dir, "scenario_runner", sr_args, stop_event))
        scenario_runner.start()
        child_pid_file.write(f"scenario_runner pid: {scenario_runner.pid}\n")

        # wait for ego to be created
        get_vehicle_by_role_name(__name__, sim_world, ego_role_name)

        if show and fps < 0:
            viewer = multiprocessing.Process(
                    target=run_pygame,
                    args=(log_dir, "view_game", ego_role_name, carla_host,
                          carla_port, 1080, 720, stop_event))
            viewer.start()
            child_pid_file.write(f"viewer pid: {viewer.pid}\n")
        else:
            pygame.init()

        control_sensor = multiprocessing.Process(
                            target=run_control,
                            args=(log_dir, "apollo_control", ego_role_name,
                                  carla_host, carla_port, apollo_host,
                                  apollo_port, stop_event))
        control_sensor.start()
        child_pid_file.write(f"control_sensor pid: {control_sensor.pid}\n")
        sensors_config = multiprocessing.Process(
                            target=run_sensors,
                            args=(log_dir, "carla_sensors",
                                  ego_role_name, carla_host,
                                  carla_port, apollo_host, apollo_port,
                                  sensor_config, stop_event))
        sensors_config.start()
        child_pid_file.write(f"sensors_config pid: {sensors_config.pid}\n")

        child_pid_file.close()
        clock = pygame.time.Clock()
        while not stop_event.is_set():
            # if not is_actor_exist(sim_world, role_name=ego_role_name):
            #     break
            if fps < 0:
                time.sleep(1)
            else:
                clock.tick_busy_loop(fps)
                sim_world.tick()

    finally:
        reset_apollo(apollo_host, dreamview_port, apollo_modules)
        if not child_pid_file.closed:
            child_pid_file.close()
        if sim_world is not None:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            sim_world.apply_settings(settings)
            destroy_all_sensors(sim_world)
        if not show:
            pygame.quit()


def external_run_scenario(arguments: argparse.Namespace()):
    ac_args, sr_args = get_args_external(arguments)
    main(ac_args, sr_args)


if __name__ == "__main__":
    ac_args, sr_args, global_stop_event = get_args()
    main(ac_args, sr_args, global_stop_event)
