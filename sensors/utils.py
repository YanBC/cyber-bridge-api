import time
from threading import Thread
import carla
from typing import List

from cyber_bridge.cyber_bridge_client import CyberBridgeClient
from sensors.base_sensor import Sensor

from sensors import *
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


def updater(s: Sensor):
    freq = s.get_frequency()
    t = 1 / 100 if freq <= 0 else 1 / freq
    while True:
        try:
            if t is not None:
                time.sleep(t)
            s.update()
        except KeyboardInterrupt:
            break


def setup_sensors(
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int,
                sensor_config: dict):
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(__name__, sim_world, ego_name)

    sensor_list = []
    for config in sensor_config:
        sensor_type = config.get("type", None)
        if sensor_type is None:
            print(f"sensor type not specified. {config}\n")
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
        sensor_list.append(tmp_sensor)

    sensor_manager = SensorManager(
            apollo_host, apollo_port, sensor_list)

    updater_list = []
    for sensor in sensor_list:
        t = Thread(target=updater, args=(sensor,))
        updater_list.append(t)

    for t in updater_list:
        t.start()

    try:
        while True:
            if not is_actor_exist(sim_world, actor_type=player_type):
                break
            sim_world.wait_for_tick()
            sensor_manager.send_apollo_msgs()
    finally:
        for sensor in sensor_list:
            sensor.destroy()
