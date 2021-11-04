import argparse
from json import load
import time
import multiprocessing
import pygame

import carla
from sensors.apollo_control import listen_and_apply_control
# from sensor_configs.test_apollo_control_module import setup_sensors
from sensor_configs.test_apollo_pnc_module import setup_sensors
from pygame_viewer import view_game
from utils import is_actor_exist, load_json_as_object
from scenario_runner import ScenarioRunner


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
    argparser.add_argument(
        'scenario',
        help='specify senario config file path')
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


def main():
    args = get_args()
    apollo_host = args.apollo
    apollo_port = args.apollo_port
    carla_host = args.carla
    carla_port = args.carla_port
    ego_role_name = args.adc
    show = args.show
    carla_timeout = args.timeout

    scenario_configs = load_json_as_object(args.scenario)
    scenario_configs.host = carla_host
    scenario_configs.port = carla_port
    scenario_configs.timeout = str(carla_timeout)
    scenario_configs.agent = None
    scenario_configs.debug = None
    scenario_configs.sync = None
    scenario_configs.openscenario = None
    scenario_configs.route = None
    scenario_configs.waitForEgo = None
    scenario_configs.randomize = None
    scenario_configs.junit = None
    scenario_configs.json = None
    scenario_configs.file = None

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

        sim_world = client.get_world()
        # settings = sim_world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.02
        # sim_world.apply_settings(settings)
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        sim_world.apply_settings(settings)

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


if __name__ == '__main__':
    main()
