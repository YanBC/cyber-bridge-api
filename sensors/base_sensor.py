import time
import carla
from cyber_bridge.base_encoder import BaseEncoder
from modules.common.proto.header_pb2 import Header


class Sensor:
    ''' information flow: Carla -> Apollo
    '''
    _apollo_channel = None
    _apollo_msgType = None
    _apollo_pbCls = None

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: carla.Sensor = None,
            freq: float = -1.,
            name: str = "") -> None:
        self._freq = freq
        self._name = name
        self.carla_sensor = carla_sensor
        if (self._apollo_channel is None or \
                self._apollo_msgType is None or \
                self._apollo_pbCls is None):
            raise NotImplementedError

        self.ego_vehicle = ego_vehicle
        self._encoder = BaseEncoder(
            self._apollo_pbCls, self._apollo_channel, self._apollo_msgType)
        self._pbCls = self._apollo_pbCls()
        self._updated = False

    def isUpdated(self):
        return self._updated

    def unsetUpdated(self):
        self._updated = False

    def update(self, time_stamp):
        self._updated = True
        raise NotImplementedError

    def get_frequency(self) -> float:
        return self._freq

    def get_name(self) -> str:
        return self._name

    def get_bytes(self) -> bytes:
        if self._updated:
            pbBytes = self._encoder.get_publish_bytes(self._pbCls)
            self._updated = False
            return pbBytes
        else:
            return b''

    def __del__(self):
        '''hanging sensors could potentially crash the carla server
        '''
        self.destroy()

    def destroy(self):
        if self.carla_sensor is not None:
            self.carla_sensor.stop()
            self.carla_sensor.destroy()
        self.carla_sensor = None

    def _get_cyber_header(self):
        header = Header()
        header.timestamp_sec = time.time()
        return header
