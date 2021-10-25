import carla
from sensors.location_sensor import LocationSensor
from sensors.chassis_sensor import ChassisSensor
from sensors.traffic_light_sensor import DummyTrafficLightSensor
from sensors.obstacles_sensor import DummyObstacleSensor
from sensors.base_sensor import SensorManager
from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)


def setup_sensors(
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int):
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(__name__, sim_world, ego_name)

    location_sensor = LocationSensor(player)
    chassis_sensor = ChassisSensor(player)
    traffic_light_sensor = DummyTrafficLightSensor(player)
    obstacles_sensor = DummyObstacleSensor(player)
    sensor_manager = SensorManager(
            apollo_host, apollo_port,
            [location_sensor, chassis_sensor, traffic_light_sensor,
            obstacles_sensor])

    while True:
        if not is_actor_exist(sim_world, actor_type=player_type):
            break
        location_sensor.update()
        chassis_sensor.update()
        traffic_light_sensor.update()
        obstacles_sensor.update()
        sensor_manager.send_apollo_msgs()
        sim_world.wait_for_tick()
