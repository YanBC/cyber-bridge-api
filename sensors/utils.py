from threading import (Thread, Event)
import carla
from typing import List
import logging
import enum

from cyber_bridge.cyber_bridge_client import CyberBridgeClient
from sensors.base_sensor import Sensor

from sensors import *
from utils import get_vehicle_by_role_name

import multiprocessing


class SensorManager:
    def __init__(
            self, host: str, port: int,
            sensors: List[Sensor]) -> None:
        self.bridge = CyberBridgeClient(
                host, port, [s._encoder for s in sensors], [])
        self.bridge.initialize()
        self.sensors = sensors

    def send_apollo_msgs(self) -> bool:
        msgList = []
        for s in self.sensors:
            # print(f"{s.get_name()}: {s.isUpdated()}")
            if s.isUpdated():
                msg = s.get_bytes()
                if len(msg) == 0:
                    continue
                msgList.append(msg)
                s.unsetUpdated()
        ret = self.bridge.send_pb_messages(msgList)
        return ret


def updater(s: Sensor, e: Event):
    freq = s.get_frequency()
    t = 1 / 100 if freq <= 0 else 1 / freq
    while True:
        try:
            if t is not None and not e.isSet():
                e.wait(t)
                s.update()
            else:
                break
        finally:
            pass


class CarlaSensorsArgs:
    def __init__(
                self,
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int,
                sensor_config: dict) -> None:
        self.ego_name = ego_name
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.apollo_host = apollo_host
        self.apollo_port = apollo_port
        self.sensor_config = sensor_config


class CarlaSensorsError(enum.Enum):
    SUCCESS = 0
    NETWORK_ERROR_CARLA = 2001
    NETWORK_ERROR_APOLLO = 2002
    UNKNOWN_ERROR = 9999
    USER_INTERRUPT = 5001


class CarlaSensorsResults:
    def __init__(
            self, err_code=CarlaSensorsError.SUCCESS) -> None:
        self.err_code = err_code


def setup_sensors(
                args: CarlaSensorsArgs,
                stop_event: multiprocessing.Event,
                output_queue: multiprocessing.Queue):
    ego_name = args.ego_name
    carla_host = args.carla_host
    carla_port = args.carla_port
    apollo_host = args.apollo_host
    apollo_port = args.apollo_port
    sensor_config = args.sensor_config

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()
    error_code = CarlaSensorsError.SUCCESS

    player, _ = get_vehicle_by_role_name(
                                    stop_event,
                                    __name__,
                                    sim_world,
                                    ego_name)
    if stop_event.is_set():
        return

    sensor_list = []
    for config in sensor_config:
        sensor_type = config.get("type", None)
        if sensor_type is None:
            logging.warning(f"sensor type not specified. {config}\n")
            continue

        if sensor_type == "CorrectedImu":
            tmp_sensor = get_CorrectedImu(player, config)
        elif sensor_type == "InsStatus":
            tmp_sensor = get_InsStatus(player, config)
        elif sensor_type == "Odometry":
            tmp_sensor = get_Odometry(player, config)
        elif sensor_type == "ChassisAlter":
            tmp_sensor = get_ChassisAlter(player, config)
        elif sensor_type == "BestPose":
            tmp_sensor = get_BestPose(player, config)
        elif sensor_type == "TrafficLightAlter":
            tmp_sensor = get_TrafficLightAlter(player, config)
        elif sensor_type == "Obstacles":
            tmp_sensor = get_Obstacles(player, config)
        elif sensor_type == "ClockSensor":
            tmp_sensor = get_ClockSensor(player, config)

        logging.info(f"{tmp_sensor.get_name()} created")
        sensor_list.append(tmp_sensor)

    sensor_manager = SensorManager(
            apollo_host, apollo_port, sensor_list)

    updater_list = []
    e = Event()
    for sensor in sensor_list:
        t = Thread(target=updater, args=(sensor, e))
        updater_list.append(t)
    for t in updater_list:
        t.start()

    try:
        while not stop_event.is_set():
            sim_world.wait_for_tick()
            sensor_manager.send_apollo_msgs()
    except Exception as e:
        # TODO
        # fine-grained classification of error_code
        # based on exception type
        error_code = CarlaSensorsError.UNKNOWN_ERROR
    finally:
        if not e.is_set():
            e.set()
        if not stop_event.is_set():
            stop_event.set()

        result = CarlaSensorsResults(error_code)
        output_queue.put(result)

        for t in updater_list:
            t.join()
        for sensor in sensor_list:
            logging.info(f"{sensor.get_name()} destroy")
            sensor.destroy()
