#!/usr/bin/env python

import sys
import os
import logging

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
from run_synchronization import SimulationSynchronization

from utils import get_vehicle_by_role_name
import enum
import multiprocessing
import carla


class IntegratedSumoError(enum.Enum):
    SUCCESS = 0
    NETWORK_ERROR_SUMO = 2001
    NETWORK_ERROR_CARLA = 2002
    UNKNOWN_ERROR = 9999
    USER_INTERRUPT = 5001


class IntegratedSumoResults:
    def __init__(
            self,
            err_code=IntegratedSumoError.SUCCESS) -> None:
        self.err_code = err_code


class IntegratedSumoArgs:
    def __init__(
        self,
        ego_name: str,
        carla_host: str,
        carla_port: int,
        sumo_cfg: str) -> None:
        self.ego_name = ego_name
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.sumo_cfg = sumo_cfg


class RunSumo:
    def __init__(self, sumo_cfg: str, client: carla.Client) -> None:
        step_length = client.get_world().get_settings().fixed_delta_seconds
        self.sumo_simulation = SumoSimulation(sumo_cfg, step_length)
        self.carla_simulation = CarlaSimulation(client)
        self.synchronization = SimulationSynchronization(
            self.sumo_simulation, self.carla_simulation)

    def tick(self):
        self.synchronization.tick()

    def __del__(self):
        if self.synchronization:
            self.synchronization.close()


def sumo_run(
            args: IntegratedSumoArgs,
            stop_event: multiprocessing.Event,
            output_queue: multiprocessing.Queue):
    ego_name = args.ego_name
    sumo_cfg = args.sumo_cfg
    carla_host = args.carla_host
    carla_port = args.carla_port

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()
    error_code = IntegratedSumoError.SUCCESS
    get_vehicle_by_role_name(
        stop_event,
        __name__,
        sim_world,
        ego_name)

    run_sumo = RunSumo(sumo_cfg, client)

    try:
        while not stop_event.is_set():
            run_sumo.tick()

    except KeyboardInterrupt:
        logging.info("keyboard interrupt received, exiting...")
        error_code = IntegratedSumoError.USER_INTERRUPT

    except Exception as e:
        logging.error(e)
        error_code = IntegratedSumoError.UNKNOWN_ERROR

    finally:
        if not stop_event.is_set():
            stop_event.set()

        result = IntegratedSumoError(error_code)
        output_queue.put(result)
