import time
import math
import json
import carla
import numpy
from typing import Tuple
from types import SimpleNamespace


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
    length =numpy.clip(2 * bbox.extent.x, 0.2, 3)
    width = numpy.clip(2 * bbox.extent.y, 0.2, 3)
    height = numpy.clip(2 * bbox.extent.z, 0.2, 3) # carla 0.9.13 has issue on bounding box of two wheels actor
    return (length, width, height)
