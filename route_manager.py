from websocket import create_connection
import json
import carla
import random
import logging
import multiprocessing
import enum
from utils import get_vehicle_by_role_name, get_gnss_sensor


class RouteManagerError(enum.Enum):
    SUCCESS = 0
    NETWORK_ERROR_SUMO = 2001
    NETWORK_ERROR_CARLA = 2002
    UNKNOWN_ERROR = 9999
    USER_INTERRUPT = 5001


class RouteManagerResults:
    def __init__(
            self,
            err_code=RouteManagerError.SUCCESS) -> None:
        self.err_code = err_code


class RouteManagerArgs:
    def __init__(
                self,
                ego_name: str,
                end_waypoint: carla.Waypoint,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                dreamview_port: int,
                loop_routing=False) -> None:
        self.ego_name = ego_name
        self.end_waypoint = end_waypoint
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.apollo_host = apollo_host
        self.dreamview_port = dreamview_port
        self.loop_routing = loop_routing


class RouteManagement:
    def __init__(self,
            actor: carla.Actor,
            world: carla.World,
            ip: str,
            port: int = 8888):
        self.url = f"ws://{ip}:{port}/websocket"
        self.ws = create_connection(self.url)
        self.actor = actor
        self.map = world.get_map()
        self.spawn_points = self.map.get_spawn_points()
        self.min_distance_to_destination = 10.0  # m, at least 10m

    def __del__(self):
        self.ws.close()

    def dir_is_same(self, actor: carla.Actor, dest: carla.Location):
        ve_dir = actor.get_transform().get_forward_vector()
        wp = self.map.get_waypoint(dest)
        wp_dir = wp.transform.get_forward_vector()
        dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z
        return True if dot_ve_wp > 0 else False

    def init_route(self,
            start_pos: carla.Location,
            end_pos: carla.Location):
        start_location = start_pos
        end_location = end_pos
        json_msg = json.dumps({
            "type": "SendRoutingRequest",
            "start": {
                "x": start_location.x,
                "y": -start_location.y,
                "z": 0
            },
            "end": {
                "x": end_location.x,
                "y": -end_location.y,
                "z": 0
            },
            "waypoint": []
        })
        logging.info(f"{json_msg}")
        self.ws.send(json_msg)
        self.current_dst = end_pos

    def route_update(self) -> None:
        actor_location = self.actor.get_location()
        if actor_location.distance(self.current_dst) < self.min_distance_to_destination \
           and self.dir_is_same(self.actor, self.current_dst):
            random.shuffle(self.spawn_points)
            if self.spawn_points[0].location != self.current_dst:
                end_loc = self.spawn_points[0].location
            else:
                end_loc = self.spawn_points[1].location

            start_loc = actor_location
            cross_locations = []
            cross_locations.append(self.current_dst)
            self.set_route_path(start_loc, end_loc, cross_locations)
            self.current_dst = end_loc  # update current destination

    def set_route_path(
            self,
            start_pos: carla.Location,
            end_pos: carla.Location,
            cross_poses=None) -> None:
        start_location = start_pos
        end_location = end_pos

        json_msg = json.dumps({
            "type": "SendRoutingRequest",
            "start": {
                "x": start_location.x,
                "y": -start_location.y,
                "z": 0
            },
            "end": {
                "x": end_location.x,
                "y": -end_location.y,
                "z": 0
            },
            "waypoint": []
        })

        cross_location = []
        for cross_pos in cross_poses:
            tmp = dict()
            tmp['x'] = cross_pos.x
            tmp['y'] = cross_pos.y
            tmp['z'] = 0
            cross_location.append(tmp)

        json_obj = json.loads(json_msg)
        json_obj['waypoint'] = cross_location

        logging.info(f"{json.dumps(json_obj)}")
        self.ws.send(json.dumps(json_obj))
        return


def route_manager(args: RouteManagerArgs,
            stop_event: multiprocessing.Event,
            output_queue: multiprocessing.Queue):
    ego_name = args.ego_name
    end_pos = args.end_waypoint.transform.location
    carla_host = args.carla_host
    carla_port = args.carla_port
    apollo_host = args.apollo_host
    dreamview_port = args.dreamview_port
    loop_routing = args.loop_routing

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    get_vehicle_by_role_name(
            stop_event, __name__, sim_world, ego_name)
    gnss_sensor = get_gnss_sensor(stop_event, sim_world)
    start_pos = gnss_sensor.get_location()
    manager_route = RouteManagement(gnss_sensor, sim_world,
                                    apollo_host, dreamview_port)
    manager_route.init_route(start_pos, end_pos)
    error_code = RouteManagerError.SUCCESS

    try:
        while not stop_event.is_set():
            sim_world.wait_for_tick()
            if loop_routing:
                manager_route.route_update()

    except Exception as e:
        if isinstance(e, KeyboardInterrupt):
            error_code = RouteManagerError.USER_INTERRUPT
        else:
            error_code = RouteManagerError.UNKNOWN_ERROR

    finally:
        if not stop_event.is_set():
            stop_event.set()

        result = RouteManagerError(error_code)
        output_queue.put(result)
