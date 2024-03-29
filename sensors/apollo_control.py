import threading
import weakref
import carla
import logging
from cyber_bridge.cyber_bridge_client import CyberBridgeClient
from cyber_bridge.apollo_control_decoder import ApolloControlDecoder
from modules.control.proto.control_cmd_pb2 import ControlCommand
from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)


def _emergency_stop():
    control = carla.VehicleControl()
    control.steer = 0.0
    control.throttle = 0.0
    control.brake = 1.0
    control.hand_brake = False
    return control


class ApolloControl:
    _apollo_channel = '/apollo/control'
    _apollo_msgType = 'apollo.control.ControlCommand'
    _apollo_pbCls = ControlCommand

    def __init__(
                self,
                ego_vehicle: carla.Vehicle,
                host: str,
                port: int) -> None:
        self.ego_vehicle = ego_vehicle
        self._decoder = ApolloControlDecoder(
                        self._apollo_pbCls,
                        self._apollo_channel,
                        self._apollo_msgType)
        self.bridge = CyberBridgeClient(
                host, port, [], [self._decoder])
        self.bridge.initialize()
        self.control = None

    @staticmethod
    def background(weak_self):
        self = weak_self()
        world = self.ego_vehicle.get_world()
        actor_type = self.ego_vehicle.type_id
        while True:
            if not is_actor_exist(world, actor_type=actor_type):
                break
            pbCls_list = self.bridge.recv_pb_messages()

            if len(pbCls_list) == 0:
                logging.warning("no control cmd received")
                # self.control = _emergency_stop()
                self.control = None
            else:
                pbControl = pbCls_list[-1]
                self.control = self._decoder.protobufToCarla(pbControl)

    @staticmethod
    def listen(weak_self):
        self = weak_self()
        world = self.ego_vehicle.get_world()
        actor_type = self.ego_vehicle.type_id
        while True:
            if not is_actor_exist(world, actor_type=actor_type):
                break
            if self.control is None:
                continue
            self.ego_vehicle.apply_control(self.control)


def listen_and_apply_control(
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int):
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, _ = get_vehicle_by_role_name(__name__, sim_world, ego_name)

    sensor = ApolloControl(player, apollo_host, apollo_port)

    weak_self = weakref.ref(sensor)
    t1 = threading.Thread(
            target=ApolloControl.background,
            args=(weak_self,),
            daemon=False)
    t2 = threading.Thread(
        target=ApolloControl.listen,
        args=(weak_self,),
        daemon=False)
    t1.start()
    t2.start()

    t1.join()
    t2.join()
