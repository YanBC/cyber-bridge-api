import argparse
from json import load
import time
import multiprocessing
import random
import os
import sys
import functools
import logging

import carla
from sensors.apollo_control import listen_and_apply_control
from sensors.utils import setup_sensors
from pygame_viewer import view_game
from utils import is_actor_exist, load_json, load_json_as_object
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
    # argparser.add_argument(
    #     'scenario',
    #     help='specify senario config file path')
    argparser.add_argument(
        '--sensor-config',
        default="sensor_configs/apollo_600_modular_testing.json",
        help='specify sensor config file path')
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
    argparser.add_argument(
        '--log-dir',
        default="./logs",
        help='where to store log files')
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


def _set_ego_vehicle_physics(vehicle):
    # Create Wheels Physics Control
    front_left_wheel  = carla.WheelPhysicsControl(radius=33.5)
    front_right_wheel = carla.WheelPhysicsControl(radius=33.5)
    rear_left_wheel   = carla.WheelPhysicsControl(radius=33.5)
    rear_right_wheel  = carla.WheelPhysicsControl(radius=33.5)
    physics_control = vehicle.get_physics_control()

    print("length=%f, width=%f, height=%f " % (vehicle.bounding_box.extent.x * 2, vehicle.bounding_box.extent.y * 2, vehicle.bounding_box.extent.z * 2))
    print("rpm=%f, moi=%f, damping=%f, use_gear_autobox=%d, gear_switch_time=%f,\
            clutch_stength=%f, mass=%f, drag_coefficient=%f, ratio= %f" % ( physics_control.max_rpm,\
    physics_control.moi,\
    physics_control.damping_rate_full_throttle,\
    physics_control.use_gear_autobox,\
    physics_control.gear_switch_time,\
    physics_control.clutch_strength,\
    physics_control.mass,\
    physics_control.drag_coefficient,\
    physics_control.final_ratio ))
    print("center x = %f, center y=%f, center z=%f" %
        (physics_control.center_of_mass.x, physics_control.center_of_mass.y, physics_control.center_of_mass.z))

    print("wheel0 = %f, wheel1 y=%f, wheel2=%f, wheel3 z=%f" %
        (physics_control.wheels[0].max_steer_angle, physics_control.wheels[1].max_steer_angle, physics_control.wheels[2].max_steer_angle, physics_control.wheels[3].max_steer_angle))

    print("radius w0 = %f, w1 y=%f, w1=%f, w3 z=%f" %
        (physics_control.wheels[0].radius , physics_control.wheels[1].radius , physics_control.wheels[2].radius, physics_control.wheels[3].radius))
    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
    physics_control.wheels = wheels
    vehicle.apply_physics_control(physics_control)
    # print("wheel0 = %f, wheel1 y=%f, wheel2=%f, wheel3 z=%f" % \
    #       (physics_control.wheels[0].max_steer_angle, physics_control.wheels[1].max_steer_angle, physics_control.wheels[2].max_steer_angle, physics_control.wheels[3].max_steer_angle))


def create_ego_vehicle(client):
    # world = client.load_world('Town03')
    world = _load_world_from_opendrive('./opendrive/CubeTown.xodr', client)
    blueprint_library = world.get_blueprint_library()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.02
    world.apply_settings(settings)
    all_default_spawn = world.get_map().get_spawn_points()
    spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()

    # spawn_location = carla.Location()
    # spawn_location.x = 58.0
    # spawn_location.y = 41.2
    # spawn_location.z = 3
    # spawn_point = world.get_map().get_waypoint(spawn_location).transform
    # actors = world.get_actors()
    # locations = [a.get_transform().location for a in actors]
    # actor_distances = [(a.x-spawn_location.x)**2 + (a.y-spawn_location.y)**2 + (a.z-spawn_location.z)**2 for a in locations]

    ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
    ego_vehicle_bp.set_attribute('role_name', 'hero')
    ego = world.spawn_actor(ego_vehicle_bp, spawn_point)
    _set_ego_vehicle_physics(ego)
    return world


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
                level=logging.DEBUG,
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


def main():
    args = get_args()
    apollo_host = args.apollo
    apollo_port = args.apollo_port
    carla_host = args.carla
    carla_port = args.carla_port
    ego_role_name = args.adc
    show = args.show
    carla_timeout = args.timeout
    sensor_config = load_json(args.sensor_config)
    log_dir = args.log_dir

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

        # Call to create ego vehicle if scenario_runner.py not run
        sim_world = create_ego_vehicle(client)

        # sim_world = client.get_world()
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        # apollo control cmd issue rate is about 50Hz
        settings.fixed_delta_seconds = 0.02
        sim_world.apply_settings(settings)

        # wait for ego vehicle
        while not is_actor_exist(sim_world, role_name=ego_role_name):
            time.sleep(1)

        if show:
            viewer = multiprocessing.Process(
                    target=run_pygame,
                    args=(log_dir, "view_game" ,ego_role_name, carla_host, carla_port, 720, 480))
            viewer.start()

        control_sensor = multiprocessing.Process(
                            target=run_control,
                            args=(log_dir, "apollo_control",
                                ego_role_name, carla_host,
                                carla_port, apollo_host, apollo_port))
        control_sensor.start()
        print("control_sensor started")
        sensors_config = multiprocessing.Process(
                            target=run_sensors,
                            args=(log_dir, "carla_sensors",
                                ego_role_name, carla_host,
                                carla_port, apollo_host, apollo_port,
                                sensor_config))
        sensors_config.start()
        print("other sensors started")

        while True:
            if not is_actor_exist(sim_world, role_name=ego_role_name):
                break
            time.sleep(1)

    finally:
        if sim_world is not None:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            sim_world.apply_settings(settings)

            destroy_all_sensors(sim_world)


if __name__ == '__main__':
    main()
