import carla
import logging
from sensors.base_sensor import Sensor
from modules.drivers.gnss.proto.ins_pb2 import InsStat


class InsStatus(Sensor):
    """
        Carla(UE) X -- East, Y -- South, Z -- Up
        Position of the vehicle reference point (VRP) in the map reference frame.
        The VRP is the center of rear axle. apollo.common.PointENU position
    """
    _apollo_channel = '/apollo/sensor/gnss/ins_stat'
    _apollo_msgType = 'apollo.drivers.gnss.InsStat'
    _apollo_pbCls = InsStat

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            freq: float = -1.,
            name: str = None) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                freq=freq,
                name=name)

    def update(self):
        self._pbCls.ins_status = 3
        self._pbCls.pos_type = 56

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_InsStatus(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        logging.error(err)
        raise ValueError
    return InsStatus(
        ego_vehicle, freq, name)
