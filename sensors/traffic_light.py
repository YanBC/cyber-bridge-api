from time import time_ns
import logging
import carla
from sensors.base_sensor import Sensor
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection, TrafficLight


def _retrieve_traffic_landmark(
            map: carla.Map,
            vehicle: carla.Vehicle,
            distance: float = 30.):
    tmp_map = map
    tmp_vehicle = vehicle
    traffic_light_landmarks = []
    waypoint_ego_near = tmp_map.get_waypoint(
            tmp_vehicle.get_location(),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving|carla.LaneType.Sidewalk))
    if waypoint_ego_near is not None:
        landmarks = waypoint_ego_near.get_landmarks(distance)
        for landmark in landmarks:
            if landmark.type == '1000001':
                if len(traffic_light_landmarks) == 0:
                    traffic_light_landmarks.append(landmark)
                elif len(traffic_light_landmarks) > 0 and landmark.id != traffic_light_landmarks[-1].id:
                    traffic_light_landmarks.append(landmark)
    else:
        logging.warning('Generate ego vehicle waypoint failed!')
    return traffic_light_landmarks


def _carla_traffic_light_to_pb(carla_tl: carla.TrafficLight):
    color = TrafficLight.Color.UNKNOWN
    # set color
    if carla_tl is not None:
        tl_state = carla_tl.get_state()
        if tl_state == carla.TrafficLightState.Red:
            color = TrafficLight.Color.RED
        elif tl_state == carla.TrafficLightState.Yellow:
            color = TrafficLight.Color.YELLOW
        elif tl_state == carla.TrafficLightState.Green:
            color = TrafficLight.Color.GREEN
        elif tl_state == carla.TrafficLightState.off:
            color = TrafficLight.Color.BLACK
        elif tl_state == carla.TrafficLightState.Unknow:
            color = TrafficLight.Color.UNKNOWN
    return color


class TrafficLightAlter(Sensor):
    _apollo_channel = '/apollo/perception/traffic_light'
    _apollo_msgType = 'apollo.perception.TrafficLightDetection'
    _apollo_pbCls = TrafficLightDetection

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            freq: float = -1.,
            name: str = None,) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                freq=freq,
                name=name)
        self._world = ego_vehicle.get_world()
        self._map = self._world.get_map()

    def update(self):
        world = self._world
        self._pbCls = self._apollo_pbCls()
        tl_lms = _retrieve_traffic_landmark(self._map, self.ego_vehicle)
        for tl_lm in tl_lms:
            traffic_light = world.get_traffic_light(tl_lm)
            rtl = TrafficLight()
            rtl.color = _carla_traffic_light_to_pb(traffic_light)
            rtl.id = tl_lm.name
            rtl.confidence = 1.0
            rtl.blink = False
            self._pbCls.traffic_light.append(rtl)

        if tl_lms is not None and len(tl_lms) > 0:
            self._pbCls.contain_lights = True
        else:
            self._pbCls.contain_lights = False

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._pbCls.header.camera_timestamp = time_ns()
        self._updated = True


def get_TrafficLightAlter(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        logging.error(err)
        raise ValueError
    return TrafficLightAlter(
        ego_vehicle, freq, name)
