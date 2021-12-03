from multiprocessing import Event
import carla
from sensors.bridge.odometry import Odometry
from sensors.location_sensor import LocationSensor
from sensors.chassis_sensor import ChassisSensor
from sensors.traffic_light_sensor import DummyTrafficLightSensor
from sensors.obstacles_sensor import DummyObstacleSensor
from sensors.base_sensor import SensorManager

from sensors.bridge.carla_sensors import GnssSensor, IMUSensor
from sensors.bridge.corrected_imu import CorrectedImu
from sensors.bridge.ins_stat import InsStatus
from sensors.bridge.odometry import Odometry
from sensors.bridge.chassis import ChassisAlter
from sensors.bridge.best_pose import BestPose
from sensors.bridge.traffic_light import TrafficLightAlter
from sensors.bridge.obstacles import Obstacles

from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)

def test_location(world, ego_vehicle, gnss_sensor):
    ego_location = ego_vehicle.get_transform().location
    gnss_location = gnss_sensor.transform.location
    print("vehicle location x=%f, y=%f, z=%f" % (ego_location.x, 
                                                 ego_location.y, 
                                                 ego_location.z))

    print("gnss location x=%f, y=%f, z=%f" % (gnss_location.x, gnss_location.y, gnss_location.z))

    gnss_transform_geolocation = world.get_map().transform_to_geolocation(carla.Location(gnss_location.x,
                                                                            -gnss_location.y,
                                                                            gnss_location.z))
    print("geolocation x=%f, y=%f, z=%f" % (gnss_transform_geolocation.latitude, gnss_transform_geolocation.longitude , gnss_transform_geolocation.altitude ))                                                                    
    print("gnss ouput lat=%f, lon=%f, alt=%f" % (gnss_sensor.lat, gnss_sensor.lon, gnss_sensor.alt))

def setup_sensors(
                ready_to_apply: Event,
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int):

    def _prompt_config_info(vehicle, gnss_sensor:GnssSensor):
        vehicle_transform = vehicle.get_transform()
        gnss_sensor_transform = gnss_sensor.transform        
        print("vehicle center location x=%f, y=%f, z=%f" % (vehicle_transform.location.x, 
                                                            vehicle_transform.location.y, 
                                                            vehicle_transform.location.z))
        print("gnss location x=%f, y=%f, z=%f" % (gnss_sensor_transform.location.x, 
                                                  gnss_sensor_transform.location.y, 
                                                  gnss_sensor_transform.location.z))

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(__name__, sim_world, ego_name)
    imu_sensor = IMUSensor(player)
    gnss_sensor = GnssSensor(player)
    location_sensor = LocationSensor(player)
    chassis_sensor = ChassisSensor(player)
    traffic_light_sensor = DummyTrafficLightSensor(player)
    obstacles_sensor = DummyObstacleSensor(player)

    corrected_imu = CorrectedImu(player, imu_sensor)
    ins_stat = InsStatus(player)
    odometry = Odometry(player, gnss_sensor)
    chassis = ChassisAlter(player, gnss_sensor)
    best_pose = BestPose(player, gnss_sensor)
    traffic_light = TrafficLightAlter(player, sim_world)
    obstacles = Obstacles(player, sim_world)
    
    if sim_world.get_settings().synchronous_mode:
        sim_world.tick()
    else:
        sim_world.wait_for_tick()

    sensor_manager = SensorManager(
            apollo_host, apollo_port,
            #  [location_sensor, chassis_sensor, traffic_light_sensor,  obstacles_sensor])
            [chassis, traffic_light,
            obstacles, corrected_imu, ins_stat, odometry, best_pose])  
             
    _prompt_config_info(player, gnss_sensor)

    while True:
        if not is_actor_exist(sim_world, actor_type=player_type):
            if imu_sensor.sensor is not None: imu_sensor.sensor.destroy()
            if gnss_sensor.sensor is not None: gnss_sensor.sensor.destroy()
            break

        sim_world.wait_for_tick()

        while not location_sensor._updated:
            location_sensor.update()
        while not chassis_sensor._updated:
            chassis_sensor.update()
        while not traffic_light_sensor._updated:
            traffic_light_sensor.update()
        while not obstacles_sensor._updated:
            obstacles_sensor.update()
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

        sensor_manager.send_apollo_msgs()

        location_sensor._updated = False
        chassis_sensor._updated = False
        traffic_light_sensor._updated = False
        obstacles_sensor._updated = False
        corrected_imu._updated = False
        ins_stat._updated = False
        odometry._updated = False
        chassis._updated = False
        best_pose._updated = False
        traffic_light._updated = False
        obstacles._updated = False

        ready_to_apply.set()
