import carla
import logging
from sensors.base_sensor import Sensor
from sensors.carla_sensors import GnssSensor, get_GnssSensor
from modules.routing.proto.routing_pb2 import (LaneWaypoint, RoutingRequest)


class RoutingReq(Sensor):
    """
        Carla(UE) X -- East, Y -- South, Z -- Up
        Position of the vehicle reference point (VRP) in the map reference frame.
        The VRP is the center of rear axle. apollo.common.PointENU position
    """
    _apollo_channel = '/apollo/routing_request'
    _apollo_msgType = 'apollo.routing.RoutingRequest'
    _apollo_pbCls = RoutingRequest

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: GnssSensor,
            destination: dict,
            freq: float = -1.,
            name: str = None) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                carla_sensor=carla_sensor,
                freq=freq,
                name=name)
        self._destination = destination

    def update(self):
        self._pbCls = self._apollo_pbCls()

        start_location_transform = self.carla_sensor.transform
        s_waypoint = LaneWaypoint()
        s_waypoint.pose.x = start_location_transform.location.x   # East
        s_waypoint.pose.y = -start_location_transform.location.y  # North
        self._pbCls.waypoint.append(s_waypoint)

        end_location_transform = self._destination
        e_waypoint = LaneWaypoint()
        e_waypoint.pose.x = end_location_transform['x']   # East
        e_waypoint.pose.y = -end_location_transform['y']  # North
        self._pbCls.waypoint.append(e_waypoint)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
        logging.info("update routing requst msg {}".format(
            self._pbCls.waypoint))


def get_RoutingReq(
        ego_vehicle: carla.Vehicle,
        config: dict,
        routing_request: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        logging.error(err)
        raise ValueError

    gnss_sensor = get_GnssSensor(ego_vehicle)
    return RoutingReq(
        ego_vehicle, gnss_sensor, routing_request, freq, name)
