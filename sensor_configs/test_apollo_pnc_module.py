import carla
from sensors.base_sensor import SensorManager
from sensors.carla_sensors import GnssSensor, IMUSensor
from sensors.corrected_imu import CorrectedImu
from sensors.ins_stat import InsStatus
from sensors.odometry import Odometry
from sensors.chassis import ChassisAlter
from sensors.best_pose import BestPose
from sensors.traffic_light import TrafficLightAlter
from sensors.obstacles import Obstacles
from sensors.clock_sensor import ClockSensor

from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)


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
    obstacles = Obstacles(player, sim_world)
    clock = ClockSensor(player)

    sensor_manager = SensorManager(
            apollo_host, apollo_port,
            [chassis, traffic_light, clock,
            obstacles, corrected_imu, ins_stat, odometry, best_pose])

    while True:
        if not is_actor_exist(sim_world, actor_type=player_type):
            break

        sim_world.wait_for_tick()

        while not corrected_imu._updated:
            corrected_imu.update()
        while not ins_stat._updated:
            ins_stat.update()
        while not odometry._updated:
            odometry.update()
        while not chassis._updated:
            chassis.update()
        while not best_pose._updated:
            best_pose.update()
        while not traffic_light._updated:
            traffic_light.update()
        while not obstacles._updated:
            obstacles.update()
        while not clock._updated:
            clock.update()

        sensor_manager.send_apollo_msgs()

        corrected_imu._updated = False
        ins_stat._updated = False
        odometry._updated = False
        chassis._updated = False
        best_pose._updated = False
        traffic_light._updated = False
        obstacles._updated = False
        clock._updated = False
