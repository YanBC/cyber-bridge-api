import carla
from sensors.location_sensor import LocationSensor
from sensors.carla_sensors import GnssSensor
from sensors.chassis import ChassisAlter
from sensors.trajectory_sensor import TrajectorySensor
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
    gnss_sensor = GnssSensor(player)

    location_sensor = LocationSensor(player)
    chassis_sensor = ChassisAlter(player, gnss_sensor)
    trajectory_sensor = TrajectorySensor(player)
    sensor_manager = SensorManager(
            apollo_host, apollo_port,
            [location_sensor, chassis_sensor, trajectory_sensor])

    while True:
        if not is_actor_exist(sim_world, actor_type=player_type):
            break
        location_sensor.update()
        chassis_sensor.update()
        trajectory_sensor.update()
        sensor_manager.send_apollo_msgs()
        sim_world.wait_for_tick()
