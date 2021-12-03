import argparse
from json import load
import time
import multiprocessing
import pygame
import random
import os
import sys

import carla
from sensors.apollo_control import listen_and_apply_control
# from sensor_configs.test_apollo_control_module import setup_sensors
from sensor_configs.test_apollo_pnc_module import setup_sensors
from pygame_viewer import view_game
from utils import is_actor_exist, load_json_as_object
from scenario_runner import ScenarioRunner

sys.path.append('./sensors/bridge')

def run_senario(args):
    scenario_runner = None
    result = True
    try:
        scenario_runner = ScenarioRunner(args)
        result = scenario_runner.run()

    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
    return not result


def get_args():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    # argparser.add_argument(
    #     'scenario',
    #     help='specify senario config file path')
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
    args = argparser.parse_args()
    return args
def _load_world_from_opendrive(xodr_path, client):
    if os.path.exists(xodr_path):
        with open(xodr_path, encoding='utf-8') as od_file:
            try:
                data = od_file.read()
            except OSError:
                print('file could not be readed.')
                sys.exit()
        print('load opendrive map %r.' % os.path.basename(xodr_path))
        vertex_distance = 2.0  # in meters
        max_road_length = 500.0 # in meters
        wall_height = 1.0      # in meters
        extra_width = 0.6      # in meters
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True))

        return world
    else:
        print('file not found.')
        return None

def create_ego_vehicle(client):
    world = client.load_world('Town01')
    # world = _load_world_from_opendrive('./CubeTown.xodr', client)
    # world = _load_world_from_opendrive('./map_conv.xodr', client)
    blueprint_library = world.get_blueprint_library()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.02
    world.apply_settings(settings)
    # spawn_point = carla.Transform(carla.Location(x=-74.32, y=-50, z=0.5), carla.Rotation(yaw=270))
    all_default_spawn = world.get_map().get_spawn_points()
    spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()
    ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
    ego_vehicle_bp.set_attribute('role_name', 'hero')
    world.spawn_actor(ego_vehicle_bp, spawn_point) 
    # world.tick()
    return world

def destroy(world):
        sensor_list = world.get_actors().filter("*sensor*")
        """Destroys all actors"""        
        for actor in sensor_list:
            if actor is not None:
                actor.destroy()

def commu_apollo_and_carla(args:argparse.Namespace):

    if args is None:
        raise RuntimeError("Error: arguments is None ")

    apollo_host = args.apolloHost
    apollo_port = args.apolloPort
    carla_host = args.carlaHost
    carla_port = args.carlaPort
    ego_role_name = args.adc
    show = args.show
    carla_timeout = args.timeout

    sim_world = None
    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(carla_timeout)

        # scenario_runner = multiprocessing.Process(
        #                     target=run_senario,
        #                     args=(scenario_configs,))
        # scenario_runner.start()

        # TODO: There should be some way to tell when the scenario is setup
        # for now, let's just wait
        # time.sleep(5)
        # print("scenario_runner started")
        # sim_world = create_ego_vehicle(client)  # Call to create ego vehicle if scenario_runner.py not run
        
        sim_world = client.get_world()
        # sim_world = client.load_world('Town01')
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.02
        sim_world.apply_settings(settings)

        # settings = sim_world.get_settings()
        # settings.synchronous_mode = False
        # settings.fixed_delta_seconds = None
        # sim_world.apply_settings(settings)

        # wait for scenario runner
        while not is_actor_exist(sim_world, role_name=ego_role_name):
            time.sleep(1)

        if show:
            viewer = multiprocessing.Process(
                    target=view_game,
                    args=(ego_role_name, carla_host, carla_port))
            viewer.start()

        sensor_ready = multiprocessing.Event()
        ready_to_tick = multiprocessing.Event()
        control_sensor = multiprocessing.Process(
                            target=listen_and_apply_control,
                            args=(sensor_ready, ready_to_tick,
                                ego_role_name, carla_host,
                                carla_port, apollo_host, apollo_port))
        control_sensor.start()
        print("control_sensor started")
        sensors_config = multiprocessing.Process(
                            target=setup_sensors,
                            args=(sensor_ready, ego_role_name, carla_host,
                            carla_port, apollo_host, apollo_port))
        sensors_config.start()
        print("other sensors started")

        # if not show:
        #     clock = pygame.time.Clock()

        while True:
            if not is_actor_exist(sim_world, role_name=ego_role_name):
                break
            # sim_world.tick()
            # # set lower limit on simulator frame rate: 10 Hz
            # if ready_to_tick.wait(1/10):
            #     ready_to_tick.clear()
            # if not show:
            #     # set uppper limit on simulator frame rate: 30 Hz
            #     clock.tick_busy_loop(30)            
            time.sleep(1)

    finally:
        if sim_world is not None:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            sim_world.apply_settings(settings)

            destroy(sim_world)

def main():
    args = get_args()
    apollo_host = args.apollo
    apollo_port = args.apollo_port
    carla_host = args.carla
    carla_port = args.carla_port
    ego_role_name = args.adc
    show = args.show
    carla_timeout = args.timeout

    # scenario_configs = load_json_as_object(args.scenario)
    # scenario_configs.host = carla_host
    # scenario_configs.port = carla_port
    # scenario_configs.timeout = str(carla_timeout)
    # scenario_configs.agent = None
    # scenario_configs.debug = None
    # scenario_configs.sync = None
    # scenario_configs.openscenario = None
    # scenario_configs.route = None
    # scenario_configs.waitForEgo = None
    # scenario_configs.randomize = None
    # scenario_configs.junit = None
    # scenario_configs.json = None
    # scenario_configs.file = None

    sim_world = None
    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(carla_timeout)

        # scenario_runner = multiprocessing.Process(
        #                     target=run_senario,
        #                     args=(scenario_configs,))
        # scenario_runner.start()

        # TODO: There should be some way to tell when the scenario is setup
        # for now, let's just wait
        # time.sleep(5)
        # print("scenario_runner started")
        # sim_world = create_ego_vehicle(client)  # Call to create ego vehicle if scenario_runner.py not run
        
        sim_world = client.get_world()
        # sim_world = client.load_world('Town01')
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.02
        sim_world.apply_settings(settings)

        # settings = sim_world.get_settings()
        # settings.synchronous_mode = False
        # settings.fixed_delta_seconds = None
        # sim_world.apply_settings(settings)

        # wait for scenario runner
        while not is_actor_exist(sim_world, role_name=ego_role_name):
            time.sleep(1)

        if show:
            viewer = multiprocessing.Process(
                    target=view_game,
                    args=(ego_role_name, carla_host, carla_port))
            viewer.start()

        sensor_ready = multiprocessing.Event()
        ready_to_tick = multiprocessing.Event()
        control_sensor = multiprocessing.Process(
                            target=listen_and_apply_control,
                            args=(sensor_ready, ready_to_tick,
                                ego_role_name, carla_host,
                                carla_port, apollo_host, apollo_port))
        control_sensor.start()
        print("control_sensor started")
        sensors_config = multiprocessing.Process(
                            target=setup_sensors,
                            args=(sensor_ready, ego_role_name, carla_host,
                            carla_port, apollo_host, apollo_port))
        sensors_config.start()
        print("other sensors started")

        # if not show:
        #     clock = pygame.time.Clock()

        while True:
            if not is_actor_exist(sim_world, role_name=ego_role_name):
                break
            # sim_world.tick()
            # # set lower limit on simulator frame rate: 10 Hz
            # if ready_to_tick.wait(1/10):
            #     ready_to_tick.clear()
            # if not show:
            #     # set uppper limit on simulator frame rate: 30 Hz
            #     clock.tick_busy_loop(30)            
            time.sleep(1)

    finally:
        if sim_world is not None:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            sim_world.apply_settings(settings)

            destroy(sim_world)

if __name__ == '__main__':
    main()
