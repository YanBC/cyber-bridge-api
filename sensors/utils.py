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


def sensor_update(s: Sensor, freq: float):
    t = 1 / 100 if freq == 0 else 1 / freq
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
                x_offset: float = 0.,
                y_offset: float = 0):
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(__name__, sim_world, ego_name)
    imu_sensor = IMUSensor(player)
    gnss_sensor = GnssSensor(player)

    corrected_imu = CorrectedImu(player, imu_sensor)
    ins_stat = InsStatus(player)
    odometry = Odometry(player, gnss_sensor, x_offset, y_offset)
    chassis = ChassisAlter(player, gnss_sensor)
    best_pose = BestPose(player, gnss_sensor)
    traffic_light = TrafficLightAlter(player, sim_world)
    obstacles = Obstacles(player, sim_world, x_offset, y_offset)
    clock = ClockSensor(player)

    sensor_manager = SensorManager(
            apollo_host, apollo_port,
            [chassis, traffic_light, clock,
            obstacles, corrected_imu, ins_stat, odometry, best_pose])

    all_threads = []
    thread_chassis = Thread(
                    target=sensor_update,
                    args=(chassis, 10))
    all_threads.append(thread_chassis)

    thread_best_pose = Thread(
                    target=sensor_update,
                    args=(best_pose, 12.5))
    all_threads.append(thread_best_pose)

    thread_odometry = Thread(
                    target=sensor_update,
                    args=(odometry, 12.5))
    all_threads.append(thread_odometry)

    thread_ins_stat = Thread(
                    target=sensor_update,
                    args=(ins_stat, 12.5))
    all_threads.append(thread_ins_stat)

    thread_corrected_imu = Thread(
                    target=sensor_update,
                    args=(corrected_imu, 0))
    all_threads.append(thread_corrected_imu)

    thread_obstacles = Thread(
                    target=sensor_update,
                    args=(obstacles, 10))
    all_threads.append(thread_obstacles)

    thread_traffic_light = Thread(
                    target=sensor_update,
                    args=(traffic_light, 10))
    all_threads.append(thread_traffic_light)

    thread_clock = Thread(
                    target=sensor_update,
                    args=(clock, 0))
    all_threads.append(thread_clock)

    for t in all_threads:
        t.start()

    while True:
        if not is_actor_exist(sim_world, actor_type=player_type):
            break
        sim_world.wait_for_tick()
        sensor_manager.send_apollo_msgs()
