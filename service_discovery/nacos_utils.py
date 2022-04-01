import xml.etree.ElementTree as ET
import json
import random
import multiprocessing
import logging
import threading
import time
from typing import Tuple
import nacos
from redis import Redis
from pottery import Redlock, TooManyExtensions

# TODO: should be in a config file or as input params
LEASE_SERVICE_NAME = "simulation.lease"
APOLLO_SERVICE_NAME = "simulation.apollo"
CARLA_SERVICE_NAME = "simulation.carla"
NACOS_NAMESPACE = "8360272a-4ca0-4e9f-8822-51b156c5c6f1"
CONFIG_GROUP = "simulation"


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
        endpoint: str,
        duration: float = 60):
    '''
    Nonblocking operation

    param:
        stop_event: global control event, set if were to stop

        endpoint: service discover endpoint, format http://ip:port

        duration: how long the servers are expected to be held
                    default to 60 seconds
    '''
    # create nacos client
    client = nacos.NacosClient(endpoint, namespace=NACOS_NAMESPACE)

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


def query_config(
        endpoint: str,
        key: str) -> dict:
    client = nacos.NacosClient(endpoint, namespace=NACOS_NAMESPACE)
    ret = client.get_config(key, CONFIG_GROUP)
    if ret is None:
        return None
    else:
        return json.loads(ret)


def publish_config(
        endpoint: str,
        key: str,
        value: str) -> bool:
    client = nacos.NacosClient(endpoint, namespace=NACOS_NAMESPACE)
    return client.publish_config(key, CONFIG_GROUP, value)


def get_scenario_config(
        endpoint: str,
        config_id: str) -> Tuple[str, ET.ElementTree]:
    config = query_config(endpoint, config_id)
    if config is None:
        return "", None

    scenario_name = config['scenario']
    xml_tree = ET.ElementTree(ET.fromstring(config['config_xml']))
    return scenario_name, xml_tree


def get_sensor_config(
        endpoint: str,
        config_id: str) -> dict:
    return query_config(endpoint, config_id)


def get_apollo_config(
        endpoint: str,
        config_id: str) -> dict:
    return query_config(endpoint, config_id)
