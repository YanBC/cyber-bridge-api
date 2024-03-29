import carla
import math
import logging
from sensors.base_sensor import Sensor
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles, PerceptionObstacle
from utils import get_actor_shape


def _get_actor_type(semantic_tag):
    type = PerceptionObstacle.Type.UNKNOWN
    if semantic_tag == 4:
        type = PerceptionObstacle.Type.PEDESTRIAN
    elif semantic_tag == 10:
        type = PerceptionObstacle.Type.VEHICLE
    elif semantic_tag == 19:
        type = PerceptionObstacle.Type.UNKNOWN_UNMOVABLE
    elif semantic_tag == 20:
        type = PerceptionObstacle.Type.UNKNOWN_MOVABLE
    return type


class Obstacles(Sensor):
    _apollo_channel = '/apollo/perception/obstacles'
    _apollo_msgType = 'apollo.perception.PerceptionObstacles'
    _apollo_pbCls = PerceptionObstacles

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            freq: float = -1.,
            name: str = None,
            x_offset: float = 0.,
            y_offset: float = 0.,
            vehicle_dis = 50,
            walker_dis = 15) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                freq=freq,
                name=name)
        self.x_offset = x_offset
        self.y_offset = y_offset
        self._vehicle_range = vehicle_dis
        self._walker_range = walker_dis

    def update(self):
        carla_world = self.ego_vehicle.get_world()
        self._pbCls = self._apollo_pbCls()

        obstacles_potencial = []
        ego_vehicle_location = self.ego_vehicle.get_transform().location

        vehicle_list = carla_world.get_actors().filter("*vehicle*")
        walker_list = carla_world.get_actors().filter("*walker.pedestrian*")
        static_obs_list = carla_world.get_actors().filter("*static.prop*")

        for actor in vehicle_list:
            if actor.get_transform().location.distance(ego_vehicle_location) <= self._vehicle_range \
                and actor.id != self.ego_vehicle.id:
                    obstacles_potencial.append(actor)

        for actor in walker_list:
            if actor.get_transform().location.distance(ego_vehicle_location) <= self._walker_range:
                obstacles_potencial.append(actor)
    
        for actor in static_obs_list:
            if actor.get_transform().location.distance(ego_vehicle_location) <= self._walker_range:
                obstacles_potencial.append(actor)

        perception_obstacle = PerceptionObstacle()
        for actor in obstacles_potencial:
            perception_obstacle.id = actor.id

            location = actor.get_location()
            perception_obstacle.position.x = location.x + self.x_offset
            perception_obstacle.position.y = -1 * location.y + self.y_offset
            perception_obstacle.position.z = location.z

            perception_obstacle.theta = -math.radians(actor.get_transform().rotation.yaw)

            velocity = actor.get_velocity()
            perception_obstacle.velocity.x = velocity.x
            perception_obstacle.velocity.y = -1 * velocity.y
            perception_obstacle.velocity.z = velocity.z

            length, width, height = get_actor_shape(actor)
            perception_obstacle.length = length
            perception_obstacle.width = width
            perception_obstacle.height = height

            # perception_obstacle.type = _get_actor_type(actor.semantic_tags[0])
            if len(actor.semantic_tags) > 0:
                perception_obstacle.type = _get_actor_type(actor.semantic_tags[0])
            else:
                perception_obstacle.type = PerceptionObstacle.Type.UNKNOWN_UNMOVABLE

            perception_obstacle.acceleration.x = actor.get_acceleration().x
            perception_obstacle.acceleration.y = -actor.get_acceleration().y
            perception_obstacle.acceleration.z = actor.get_acceleration().z
            perception_obstacle.anchor_point.x = actor.get_location().x
            perception_obstacle.anchor_point.y = -actor.get_location().y
            perception_obstacle.anchor_point.z = actor.get_location().z

            self._pbCls.perception_obstacle.append(perception_obstacle)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_Obstacles(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
        x_offset = config['x_offset']
        y_offset = config['y_offset']
        vehicle_distance = config['vehicle_distance']
        walker_distance = config['walker_distance']
    except KeyError as err:
        logging.error(err)
        raise ValueError
    return Obstacles(
        ego_vehicle, freq, name, x_offset,
        y_offset, vehicle_distance, walker_distance)
