import carla
import time
import logging
from sensors.base_sensor import Sensor
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose, NARROW_INT, WGS84
from sensors.carla_sensors import GnssSensor, get_GnssSensor


class BestPose(Sensor):
    _apollo_channel = '/apollo/sensor/gnss/best_pose'
    _apollo_msgType = 'apollo.drivers.gnss.GnssBestPose'
    _apollo_pbCls = GnssBestPose

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: GnssSensor,
            freq: float = -1.,
            name: str = None) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                carla_sensor=carla_sensor,
                freq=freq,
                name=name)
        self._begining_time = time.time()

    def update(self):
        self._pbCls.measurement_time = self.carla_sensor.timestamp + self._begining_time
        self._pbCls.sol_type = NARROW_INT
        self._pbCls.latitude = self.carla_sensor.lat
        self._pbCls.longitude = self.carla_sensor.lon
        self._pbCls.height_msl = self.carla_sensor.alt
        self._pbCls.datum_id = WGS84
        self._pbCls.latitude_std_dev = 0.0
        self._pbCls.longitude_std_dev = 0.0
        self._pbCls.height_std_dev = 0.0

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_BestPose(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        logging.error(err)
        raise ValueError
    gnss_sensor = get_GnssSensor(ego_vehicle)
    return BestPose(
        ego_vehicle, gnss_sensor, freq, name)
