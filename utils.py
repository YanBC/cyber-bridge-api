import time
import math
import carla
from typing import Tuple


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
