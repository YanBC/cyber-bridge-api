import time
from threading import (Thread, Event)
import carla
from typing import List
import logging

from cyber_bridge.cyber_bridge_client import CyberBridgeClient
from sensors.base_sensor import Sensor

from sensors import *
from sensors.routing_request import RoutingReq, get_RoutingReq
from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)


class SensorManager:
    def __init__(
            self, host:str, port:int,
            sensors:List[Sensor]) -> None:
        self.bridge = CyberBridgeClient(
                host, port, [s._encoder for s in sensors], [])
        self.bridge.initialize()
        self.sensors = sensors

    def send_apollo_msgs(self) -> bool:
        msgList = []
        for s in self.sensors:
            if s.isUpdated:
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
    # while True:
    #     try:
    #         if t is not None:
    #             time.sleep(t)
    #         s.update()
    #     except KeyboardInterrupt:
    #         break
    while True:
        try:
            if t is not None and not e.isSet():
                e.wait(t)
                s.update()
            else:
                break
        finally:
            pass

def setup_sensors(
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int,
                sensor_config: dict,
                routing_request: dict):
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(__name__, sim_world, ego_name)

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
        elif sensor_type == "RoutingReq":
            tmp_sensor = get_RoutingReq(player, config, routing_request)

        logging.info(f"{tmp_sensor.get_name()} created")
        sensor_list.append(tmp_sensor)

    sensor_manager = SensorManager(
            apollo_host, apollo_port, sensor_list)

    updater_list = []
    e = Event()
    for sensor in sensor_list:
        if not isinstance(sensor, RoutingReq):
            t = Thread(target=updater, args=(sensor, e))
            updater_list.append(t)
        else:  # routing request sent only once
            sensor.update()
            print("Sensor into={}{}".format(sensor, sensor._updated))
    for t in updater_list:
        t.start()

    try:
        while True:
            if not is_actor_exist(sim_world, actor_type=player_type):
                logging.info("ego vehicle no longer exist")
                logging.info("exiting...")
                break
            sim_world.wait_for_tick()
            sensor_manager.send_apollo_msgs()
    finally:
        e.set()
        for t in updater_list:
            t.join()
        for sensor in sensor_list:
            logging.info(f"{sensor.get_name()} destroy")
            sensor.destroy()
