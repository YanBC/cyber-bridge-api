import time
import multiprocessing
import logging
import pygame
import xml.etree.ElementTree as ET

import carla
from pygame_viewer import view_game
from sensors.apollo_control import (
    listen_and_apply_control, ApolloControlArgs,
    ApolloControlError, ApolloControlResults)
from sensors.utils import (
    setup_sensors, CarlaSensorsArgs,
    CarlaSensorsError, CarlaSensorsResults)
from scenario_runner.scenario_runner import (
    ScenarioRunArgs, ScenarioRunError,
    scenario_run, ScenarioRunResults)
from utils import (
    load_json, get_vehicle_by_role_name,
    logging_wrapper)
from srunner.tools.scenario_parser \
    import ScenarioConfigurationParser as SrCfgP
from dreamview_api import setup_apollo, reset_apollo
from db.error_codes import ErrorCodes
from route_manager import RouteManagerArgs, RouteManagerResults, route_manager
from sumo.sumo_runner import IntegratedSumoArgs, IntegratedSumoResults, sumo_run


def destroy_all_sensors(world):
    sensor_list = world.get_actors().filter("*sensor*")
    """Destroys all actors"""
    for actor in sensor_list:
        if actor is not None:
            actor.destroy()


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


# @logging_wrapper
def startup_simulation(*args, **kwargs):
    return start_simulation(*args, **kwargs)


@logging_wrapper
def run_sumo(*args, **kwargs):
    return sumo_run(*args, **kwargs)


@logging_wrapper
def run_route_manager(*args, **kwargs):
    return route_manager(*args, **kwargs)


class SimulationArgs:
    def __init__(self, ) -> None:
        pass


class SimulationResult:
    err_code = ErrorCodes.SUCCESS
    criteria = []

    def __init__(self) -> None:
        pass

    def set_err_code(self, err_code:ErrorCodes):
        self.err_code = err_code


# TODO
def NewSimulationResult(
        scenario_run_result: ScenarioRunResults,
        control_sensor_result: ApolloControlResults,
        carla_sensors_result: CarlaSensorsResults,
        route_manager_result: RouteManagerResults,
        integrated_sumo_result: IntegratedSumoResults
    ) -> SimulationResult:
    return SimulationResult()


# synchroneous mode only
# single scenario only
def start_simulation(
            stop_event: multiprocessing.Event,
            apollo_host: str,
            apollo_port: int,
            dreamview_port: int,
            carla_host: str,
            carla_port: int,
            scenario_name: str,
            # scenario_config example:
            # scenario_configs/781_stop_at_fix_location_cfg.xml
            scenario_config_tree: ET.ElementTree,
            # sensor_config exmaple:
            # sensor_configs/apollo_600_modular_testing.json
            sensor_config: dict,
            # apollo_config example:
            # apollo_configs/pnc_testing.json
            apollo_config: dict,
            fps=50,
            log_dir='./log',
            ego_role_name='hero',
            carla_timeout=20.0,
            show=False,
            enable_sumo=False,
            sumo_cfg=None) -> SimulationResult:
    #########################
    # Parse arguments
    #########################
    result = SimulationResult()
    try:
        dreamview_mode = apollo_config['mode']
        apollo_modules = apollo_config["modules"]

        scenario_configs = SrCfgP.parse_customed_scenario_configuration_from_tree(
                    scenario_name, scenario_config_tree)
        try:
            dst_x = scenario_configs[0].destination.x
            dst_y = scenario_configs[0].destination.y
            dst_z = scenario_configs[0].destination.z
        except Exception:
            dst_x = dst_y = dst_z = None
        ego_config = scenario_configs[0].ego_vehicles[0]
        srt_x = ego_config.transform.location.x
        srt_y = ego_config.transform.location.y
        srt_z = ego_config.transform.location.z

        kv_map_names = load_json("./kv_mappings/maps.json")
        kv_vehicle_names = load_json("./kv_mappings/vehicles.json")
        carla_map = scenario_configs[0].town
        carla_vehicle = scenario_configs[0].ego_vehicles[0].model
        apollo_map = kv_map_names.get(carla_map, None)
        if apollo_map is None:
            logging.error(f"No Apollo map for {carla_map}")
            result.set_err_code(ErrorCodes.CONFIG_ERROR)
            return result
        apollo_vehicle = kv_vehicle_names.get(carla_vehicle, None)
        if apollo_vehicle is None:
            logging.error(f"No Apollo vehicle for {apollo_vehicle}")
            result.set_err_code(ErrorCodes.CONFIG_ERROR)
            return result
    except Exception as e:
        if isinstance(e, KeyError):
            logging.error(f"Key error in configuration, {e}")
        else:
            logging.error(f"Unknown error in configuration, {e}")
        result.set_err_code(ErrorCodes.CONFIG_ERROR)
        return result

    sim_world = None
    child_pid_file = open("pids.txt", "w")

    #########################
    # Setup
    #########################
    loop_routing = False
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
            if dst_x == 0 and dst_y == 0 and dst_z == 0:
                loop_routing = True
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
            result.set_err_code(ErrorCodes.APOLLO_BOOSTRAP_ERROR)
            return result
    except Exception as e:
        logging.error(f"fail in simulation setup, {e}")
        result.set_err_code(ErrorCodes.UNKNOWN_ERROR)
        return result

    #########################
    # Process
    #########################
    scenario_run_result = ScenarioRunResults()
    control_sensor_result = ApolloControlResults()
    carla_sensors_result = CarlaSensorsResults()
    route_manager_result = RouteManagerResults()
    integrated_sumo_result = IntegratedSumoResults()
    try:
        scenario_runner_args = ScenarioRunArgs(
                host=carla_host,
                port=carla_port,
                timeout=carla_timeout,
                scenario_configs=scenario_configs
        )
        scenario_runner_queue = multiprocessing.Queue()
        scenario_runner = multiprocessing.Process(
                        target=run_scenario,
                        args=(log_dir, "scenario_runner",
                              scenario_runner_args,
                              stop_event, scenario_runner_queue))
        scenario_runner.start()
        child_pid_file.write(f"scenario_runner pid: {scenario_runner.pid}\n")

        # wait for ego to be created
        get_vehicle_by_role_name(stop_event, __name__, sim_world, ego_role_name)
        if stop_event.is_set():
            scenario_run_result = scenario_runner_queue.get()
            result = NewSimulationResult(
                    scenario_run_result,
                    control_sensor_result,
                    carla_sensors_result,
                    route_manager_result,
                    integrated_sumo_result)
            return result

        if show and fps < 0:
            viewer = multiprocessing.Process(
                    target=run_pygame,
                    args=(log_dir, "view_game", ego_role_name, carla_host,
                          carla_port, 1080, 720, stop_event))
            viewer.start()
            child_pid_file.write(f"viewer pid: {viewer.pid}\n")
        else:
            pygame.init()

        if enable_sumo:
            if not sumo_cfg:
                logging.error("Missing sumo config file")
            else:
               integrate_sumo_args = IntegratedSumoArgs(
                    ego_name=ego_role_name,
                    carla_host=carla_host,
                    carla_port=carla_port,
                    sumo_cfg=sumo_cfg
               )
               integrate_sumo_queue = multiprocessing.Queue()
               integrate_sumo = multiprocessing.Process(
                                    target=run_sumo,
                                    args=(log_dir, "integrate_sumo",
                                    integrate_sumo_args,
                                    stop_event,
                                    integrate_sumo_queue))
               integrate_sumo.start()
               child_pid_file.write(f"control_sensor pid: {integrate_sumo.pid}\n")

        control_sensor_args = ApolloControlArgs(
                ego_name=ego_role_name,
                carla_host=carla_host,
                carla_port=carla_port,
                apollo_host=apollo_host,
                apollo_port=apollo_port
        )
        control_sensor_queue = multiprocessing.Queue()
        control_sensor = multiprocessing.Process(
                            target=run_control,
                            args=(log_dir, "apollo_control",
                                  control_sensor_args,
                                  stop_event,
                                  control_sensor_queue))
        control_sensor.start()
        child_pid_file.write(f"control_sensor pid: {control_sensor.pid}\n")

        carla_sensors_args = CarlaSensorsArgs(
                ego_name=ego_role_name,
                carla_host=carla_host,
                carla_port=carla_port,
                apollo_host=apollo_host,
                apollo_port=apollo_port,
                sensor_config=sensor_config
        )
        carla_sensors_queue = multiprocessing.Queue()
        carla_sensors = multiprocessing.Process(
                            target=run_sensors,
                            args=(log_dir, "carla_sensors",
                                  carla_sensors_args,
                                  stop_event,
                                  carla_sensors_queue))
        carla_sensors.start()
        child_pid_file.write(f"carla_sensors pid: {carla_sensors.pid}\n")

        route_manager_args = RouteManagerArgs(
            ego_name=ego_role_name,
            end_waypoint=end_waypoint,
            carla_host=carla_host,
            carla_port=carla_port,
            apollo_host=apollo_host,
            dreamview_port=dreamview_port,
            loop_routing=loop_routing
        )
        route_manager_queue = multiprocessing.Queue()
        route_manager = multiprocessing.Process(
                            target=run_route_manager,
                            args=(log_dir, "route_manager",
                                  route_manager_args,
                                  stop_event,
                                  route_manager_queue))
        route_manager.start()
        child_pid_file.write(f"route manager pid: {route_manager.pid}\n")
        child_pid_file.close()

        clock = pygame.time.Clock()
        while not stop_event.is_set():
            if fps < 0:
                time.sleep(1)
            else:
                clock.tick_busy_loop(fps)
                sim_world.tick()

    except Exception as e:
        logging.error(e)
        result.set_err_code(ErrorCodes.UNKNOWN_ERROR)
        return result

    finally:
        if not stop_event.is_set():
            stop_event.set()

        scenario_run_result = scenario_runner_queue.get()
        control_sensor_result = control_sensor_queue.get()
        carla_sensors_result = carla_sensors_queue.get()
        route_manager_result = route_manager_queue.get()
        integrated_sumo_result = integrate_sumo_queue.get() if enable_sumo else None

        result = NewSimulationResult(
                scenario_run_result=scenario_run_result,
                control_sensor_result=control_sensor_result,
                carla_sensors_result=carla_sensors_result,
                route_manager_result=route_manager_result,
                integrated_sumo_result=integrated_sumo_result)

    #########################
    # Clean up
    #########################
    # Try to clean up. Best effort only.
    # If it fails, it fials.
    try:
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
    except Exception as e:
        logging.error(f"error in simulation cleanup, {e}")

    return result
