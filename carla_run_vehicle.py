import argparse
import time
import multiprocessing

import carla
from sensors.apollo_control import listen_and_apply_control
# from sensor_configs.test_apollo_control_module import setup_sensors
from sensor_configs.test_apollo_pnc_module import setup_sensors
from pygame_viewer import view_game
from utils import is_actor_exist


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
        help='ego vehicle role name')
    argparser.add_argument(
        '--show',
        action='store_true',
        help='display adc in pygame')
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

    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(4.0)

        sim_world = client.get_world()
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        sim_world.apply_settings(settings)

        while not is_actor_exist(sim_world, role_name=ego_role_name):
            time.sleep(1)

        # setup sensors here
        control_sensor = multiprocessing.Process(
                            target=listen_and_apply_control,
                            args=(ego_role_name, carla_host,
                            carla_port, apollo_host, apollo_port))
        control_sensor.start()
        sensors_config = multiprocessing.Process(
                            target=setup_sensors,
                            args=(ego_role_name, carla_host,
                            carla_port, apollo_host, apollo_port))
        sensors_config.start()

        if show:
            viewer = multiprocessing.Process(
                    target=view_game,
                    args=(ego_role_name, carla_host, carla_port))
            viewer.start()

        while True:
            if not is_actor_exist(sim_world, role_name=ego_role_name):
                break
            sim_world.tick()

    finally:
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        sim_world.apply_settings(settings)


if __name__ == '__main__':
    main()
