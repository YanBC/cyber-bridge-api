import carla
import math
from sensors.base_sensor import Sensor
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles, PerceptionObstacle


class Obstacles(Sensor):
    _apollo_channel = '/apollo/perception/obstacles'
    _apollo_msgType = 'apollo.perception.PerceptionObstacles'
    _apollo_pbCls = PerceptionObstacles

    def __init__(self, ego_vehicle: carla.Vehicle, world: carla.World, vehicle_dis = 50, walker_dis = 15) -> None:
        super().__init__(ego_vehicle)
        self._world = world
        self._vehicle_range = vehicle_dis
        self._walker_range = walker_dis

    def update(self):
        def _get_actor_type(semantic_tag):
            type = PerceptionObstacle.Type.UNKNOWN
            if semantic_tag == 4:
                type = PerceptionObstacle.Type.PEDESTRIAN
            elif semantic_tag == 10:
                type = PerceptionObstacle.Type.VEHICLE
            elif semantic_tag == 20:
                type = PerceptionObstacle.Type.UNKNOWN_MOVABLE
            return type

        def _is_valid_obstacle(semantic_tag):
            if semantic_tag == 4 or semantic_tag == 10:
                return True
            return False

        self._pbCls = self._apollo_pbCls()

        obstacles_potencial = []
        ego_vehicle_location = self.ego_vehicle.get_transform().location

        vehicle_list = self._world.get_actors().filter("*vehicle*")
        walker_list = self._world.get_actors().filter("*walker.pedestrian*")

        for actor in vehicle_list:
            if actor.get_transform().location.distance(ego_vehicle_location) <= self._vehicle_range \
                and actor.id != self.ego_vehicle.id:
                        obstacles_potencial.append(actor)

        for actor in walker_list:
            if actor.get_transform().location.distance(ego_vehicle_location) <= self._walker_range:
                obstacles_potencial.append(actor)

        perception_obstacle = PerceptionObstacle()
        for actor in obstacles_potencial:
            perception_obstacle.id = actor.id
            perception_obstacle.position.x = actor.get_location().x
            perception_obstacle.position.y = -actor.get_location().y
            perception_obstacle.position.z = actor.get_location().z
            perception_obstacle.theta = math.radians(-self.ego_vehicle.get_transform().rotation.yaw - 90)
            perception_obstacle.velocity.x = actor.get_velocity().x
            perception_obstacle.velocity.y = -actor.get_velocity().y
            perception_obstacle.velocity.z = actor.get_velocity().z
            perception_obstacle.length = actor.bounding_box.extent.x * 2
            perception_obstacle.width = actor.bounding_box.extent.y * 2
            perception_obstacle.height = actor.bounding_box.extent.z * 2

            perception_obstacle.type = _get_actor_type(actor.semantic_tags[0])

            perception_obstacle.acceleration.x = actor.get_acceleration().x
            perception_obstacle.acceleration.y = -actor.get_acceleration().y
            perception_obstacle.acceleration.z = actor.get_acceleration().z
            perception_obstacle.anchor_point.x = actor.get_location().x
            perception_obstacle.anchor_point.y = -actor.get_location().y
            perception_obstacle.anchor_point.z = actor.get_location().z

            self._pbCls.perception_obstacle.append(perception_obstacle)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
