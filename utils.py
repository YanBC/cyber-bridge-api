import time
import math
import json
import carla
import numpy
from typing import Tuple
from types import SimpleNamespace
import threading
import multiprocessing
import random
import nacos
from redis import Redis
from pottery import Redlock, TooManyExtensions
import logging


def get_vehicle_by_role_name(
                module_name: str,
                carla_world: carla.World,
                role_name: str) -> Tuple[carla.Vehicle, str]:
    player = None
    player_type = None
    while player is None:
        print("Waiting for the ego vehicle...")
        time.sleep(1)
        possible_vehicles = carla_world.get_actors().filter('vehicle.*')
        for vehicle in possible_vehicles:
            if vehicle.attributes['role_name'] == role_name:
                print(f"{module_name}: ego vehicle found")
                player = vehicle
                player_type = player.type_id
                break
    return player, player_type


def is_actor_exist(
                carla_world: carla.World,
                role_name: str = None,
                actor_type: str = None) -> bool:
    if actor_type is not None:
        return len(carla_world.get_actors().filter(actor_type)) >= 1
    elif role_name is not None:
        actors = carla_world.get_actors()
        for actor in actors:
            actor_role_name = actor.attributes.get('role_name', None)
            if actor_role_name is None:
                continue
            if actor_role_name == role_name:
                return True
        else:
            return False
    else:
        raise ValueError


def cal_distance(a: carla.Location, b: carla.Location) -> float:
    sqare = (a.x - b.x)^2 + (a.y - b.y)^2 + (a.z - b.z)^2
    return math.sqrt(sqare)


def load_json(filepath: str) -> dict:
    with open(filepath) as f:
        data = json.load(f)
    return data


def load_json_as_object(filepath: str) -> object:
    data_dict = load_json(filepath)
    return SimpleNamespace(**data_dict)


def pretty_json(data: str) -> str:
    json_data = json.loads(data)
    ret = json.dumps(json_data, indent=4, sort_keys=True)
    return ret


# Available on carla.Vehicle, carla.Walker
# # carla.EnvironmentObject and carla.Junction
# Returns (length,, width, height)
# please refer to: https://github.com/carla-simulator/carla/issues/3670
def get_actor_shape(a: carla.Actor) -> Tuple[float]:
    bbox = a.bounding_box
    # bbox = a.get_world().get_level_bbs(a.type_id)
    # vertices = bbox.get_local_vertices()
    # print("bounding box={}".format(bbox))
    # v_min = vertices[0]
    # v_max = vertices[7]
    # length = v_max.x - v_min.x
    # width = v_max.y - v_min.y
    # height = v_max.z = v_min.z
    length = numpy.clip(2 * bbox.extent.x, 0.2, 3)
    width = numpy.clip(2 * bbox.extent.y, 0.2, 3)
    height = numpy.clip(2 * bbox.extent.z, 0.2, 3) # carla 0.9.13 has issue on bounding box of two wheels actor
    return (length, width, height)


def longitudinal_offset(wp: carla.Waypoint, offset: float) -> carla.Location:
    position_yaw = wp.transform.rotation.yaw
    offset_angel = position_yaw
    offset_location = carla.Location(
        offset * math.cos(math.radians(offset_angel)),
        offset * math.sin(math.radians(offset_angel)))
    return wp.transform.location + offset_location


#######################
# service discovery
#######################
# TODO: should be in a config file or as input params
LEASE_SERVICE_NAME = "simulation.lease"
APOLLO_SERVICE_NAME = "simulation.apollo"
CARLA_SERVICE_NAME = "simulation.carla"
NACOS_NAMESPACE = "8360272a-4ca0-4e9f-8822-51b156c5c6f1"


def hovor_resource(
        stop_event: multiprocessing.Event,
        redlock: Redlock):
    while not stop_event.wait(1):
        if 1 < redlock.locked() < 5:
            try:
                redlock.extend()
            except TooManyExtensions:
                logging.error("lease expires")
                break
    stop_event.set()
    redlock.release()


def acquire_servers(
        stop_event: multiprocessing.Event,
        s_discovery: str,
        duration: float = 60):
    '''
    Nonblocking operation

    param:
        stop_event: global control event, set if were to stop

        s_discovery: service discover endpoint, format http://ip:port

        duration: how long the servers are expected to be used
                    default to 60 seconds
    '''
    # create nacos client
    client = nacos.NacosClient(s_discovery, namespace=NACOS_NAMESPACE)

    # get lease server
    services = client.list_naming_instance(LEASE_SERVICE_NAME, healthy_only=False)
    services_hosts = services["hosts"]
    if len(services_hosts) == 0:
        raise RuntimeError("No lease services")
    lease_redis = []
    for lease_host in services_hosts:
        lease_url = f"redis://{lease_host['ip']}:{lease_host['port']}"
        lease_redis.append(Redis.from_url(lease_url))

    # get available apollo servers
    services = client.list_naming_instance(APOLLO_SERVICE_NAME, healthy_only=False)
    services_hosts = services["hosts"]
    if len(services_hosts) == 0:
        raise RuntimeError("No apollo services")

    random.shuffle(services_hosts)
    for apollo_service in services_hosts:
        apollo_id = apollo_service['instanceId']
        apollo_lock = Redlock(
                        key=apollo_id,
                        masters=lease_redis,
                        auto_release_time=duration)
        if apollo_lock.acquire(blocking=False):
            apollo_host = apollo_service["ip"]
            break
        else:
            time.sleep(random.random())
    else:
        raise RuntimeError("No available apollo services")

    apollo_hover_thread = threading.Thread(
                            target=hovor_resource,
                            args=(stop_event, apollo_lock))
    apollo_hover_thread.start()

    # get available carla servers
    services = client.list_naming_instance(CARLA_SERVICE_NAME, healthy_only=False)
    services_hosts = services["hosts"]
    if len(services_hosts) == 0:
        raise RuntimeError("No carla services")

    random.shuffle(services_hosts)
    for carla_service in services_hosts:
        carla_id = carla_service['instanceId']
        carla_lock = Redlock(
                        key=carla_id,
                        masters=lease_redis,
                        auto_release_time=duration)
        if carla_lock.acquire(blocking=False):
            carla_host = carla_service["ip"]
            break
        else:
            time.sleep(random.random())
    else:
        raise RuntimeError("No available apollo services")

    carla_hover_thread = threading.Thread(
                            target=hovor_resource,
                            args=(stop_event, carla_lock))
    carla_hover_thread.start()

    return apollo_host, carla_host
