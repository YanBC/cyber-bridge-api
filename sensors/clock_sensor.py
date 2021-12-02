import time
import carla
from sensors.base_sensor import Sensor
from cyber.proto.clock_pb2 import Clock


class ClockSensor(Sensor):
    _apollo_channel = '/clock'
    _apollo_msgType = 'apollo.cyber.proto.Clock'
    _apollo_pbCls = Clock

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
        # in nano seconds
        self._pbCls.clock = int(time.time() * 10**9)
        self._updated = True


def get_ClockSensor(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        print(err)
        raise ValueError
    return ClockSensor(
        ego_vehicle, freq, name)
