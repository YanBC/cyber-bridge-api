import math
import carla
import time
from sensors.base_sensor import Sensor
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose, NARROW_INT, WGS84
from sensors.bridge.carla_sensors import GnssSensor

class BestPose(Sensor):
    _apollo_channel = '/apollo/sensor/gnss/best_pose'
    _apollo_msgType = 'apollo.drivers.gnss.GnssBestPose'
    _apollo_pbCls = GnssBestPose

    def __init__(self, ego_vehicle: carla.Vehicle, gnss_sensor:GnssSensor) -> None:
        super().__init__(ego_vehicle)
        self._gnss_sensor = gnss_sensor
        self._begining_time = time.time()

    def update(self):        
        self._pbCls.measurement_time = self._gnss_sensor.timestamp + self._begining_time
        self._pbCls.sol_type = NARROW_INT
        self._pbCls.latitude = self._gnss_sensor.lat
        self._pbCls.longitude = self._gnss_sensor.lon
        self._pbCls.height_msl = self._gnss_sensor.alt
        self._pbCls.datum_id = WGS84
        self._pbCls.latitude_std_dev = 0.0
        self._pbCls.longitude_std_dev = 0.0
        self._pbCls.height_std_dev = 0.0

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
