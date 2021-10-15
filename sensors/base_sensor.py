import time
import weakref
from typing import List
from google.protobuf.message import Message as pbMessage

import carla
from cyber_bridge_client import CyberBridgeClient
from modules.common.proto.header_pb2 import Header
from encoders.base_encoder import BaseEncoder


class Sensor:
    ''' information flow: Carla -> Apollo
    '''
    _apollo_channel = None
    _apollo_msgType = None
    _apollo_pbCls = None

    def __init__(self, ego_vehicle: carla.Vehicle) -> None:
        self.sensor = None
        if (self._apollo_channel is None or \
                self._apollo_msgType is None or \
                self._apollo_pbCls is None):
            raise NotImplementedError

        self.ego_vehicle = ego_vehicle
        self._encoder = BaseEncoder(
            self._apollo_pbCls, self._apollo_channel, self._apollo_msgType)
        self.ego_vehicle.get_world().on_tick(self.update)
        self._pbCls = self._apollo_pbCls()
        self._updated = False

    def update(self, time_stamp):
        '''this method is invoked on every server tick
        '''
        self._updated = True
        raise NotImplementedError

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
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
        self.sensor = None

    def _get_cyber_header(self):
        header = Header()
        header.timestamp_sec = time.time()
        return header


class CarlaSensor(Sensor):
    '''
    This class is for demonstration purpose only;
    Any child class should inherits from the Sensor
    class above.
    '''
    _carla_bp = None

    def __init__(self, ego_vehicle: carla.Vehicle, transform: carla.Transform = None) -> None:
        if (self._apollo_channel is None or \
                self._apollo_msgType is None or \
                self._apollo_pbCls is None or \
                self._carla_bp is None):
            raise NotImplementedError

        self.ego_vehicle = ego_vehicle
        self._encoder = BaseEncoder(
            self._apollo_pbCls, self._apollo_channel, self._apollo_msgType)
        world = self.ego_vehicle.get_world()
        bp = world.get_blueprint_library().find(self._carla_bp)
        if transform is None:
            transform = carla.Transform()
        self.sensor = world.spawn_actor(bp, transform, attach_to=self.ego_vehicle)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CarlaSensor.update(weak_self, event))

    @staticmethod
    def update(weak_self, event):
        '''this method is invoked on every server tick
        '''
        raise NotImplementedError

    def _get_pbCls(self) -> pbMessage:
        raise NotImplementedError


class SensorManager:
    def __init__(
            self, host:str, port:int,
            sensors:List[Sensor]) -> None:
        self.bridge = CyberBridgeClient(
                host, port, [s._encoder for s in sensors], [])
        self.bridge.initialize()
        self.sensors = sensors

    def send_apollo_msgs(self) -> bool:
        msgList = []
        for s in self.sensors:
            msg = b''
            while len(msg) == 0:
                msg = s.get_bytes()
            msgList.append(msg)
        ret = self.bridge.send_pb_messages(msgList)
        return ret
