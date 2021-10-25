import carla
from sensors.base_sensor import Sensor
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection, TrafficLight
from utils import cal_distance


class TrafficLightSensor(Sensor):
    _apollo_channel = '/apollo/perception/traffic_light'
    _apollo_msgType = 'apollo.perception.TrafficLightDetection'
    _apollo_pbCls = TrafficLightDetection

    def __init__(
            self, ego_vehicle: carla.Vehicle,
            radius: int = 50) -> None:
        super().__init__(ego_vehicle)
        # only detect traffic lights within radius (unit: meter)
        self._radius = radius
        self._sim_world = ego_vehicle.get_world()

    def _traffic_light_carla_to_pb(
            self, carla_tl: carla.TrafficLight) -> TrafficLight:
        pb_tl = TrafficLight()
        # set color
        tl_state = carla_tl.get_state()
        if tl_state == carla.TrafficLightState.Red:
            pb_tl.color = TrafficLight.Color.RED
        elif tl_state == carla.TrafficLightState.Yellow:
            pb_tl.color = TrafficLight.Color.YELLOW
        elif tl_state == carla.TrafficLightState.Green:
            pb_tl.color = TrafficLight.Color.CREEN
        elif tl_state == carla.TrafficLightState.off:
            pb_tl.color = TrafficLight.Color.BLACK
        elif tl_state == carla.TrafficLightState.Unknow:
            pb_tl.color = TrafficLight.Color.UNKNOWN
        return pb_tl

    def update(self):
        ego_location = self.ego_vehicle.get_location()
        all_traffic_lights = self._sim_world.get_actors().filter('traffic.traffic_light')

        pb_tld = TrafficLightDetection()
        for tl in all_traffic_lights:
            tl_location = tl.get_location()
            distance = cal_distance(ego_location, tl_location)
            if distance < self._radius:
                pb_tl = self._traffic_light_carla_to_pb(tl)
                pb_tld.traffic_light.append(pb_tl)

        pb_tld.contain_lights = True \
            if len(pb_tld.traffic_light) > 0 else False

        self._pbCls.CopyFrom(pb_tld)
        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


class DummyTrafficLightSensor(TrafficLightSensor):
    def update(self):
        self._pbCls = TrafficLightDetection()
        self._pbCls.contain_lights = False
        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
