import time
import threading
import weakref
import signal
import carla
from cyber_bridge_client import CyberBridgeClient
from modules.control.proto.control_cmd_pb2 import ControlCommand
from decoders.apollo_control_decoder import ApolloControlDecoder


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
        while True:
            pbCls_list = self.bridge.recv_pb_messages()
            if len(pbCls_list) == 0:
                continue
            pbControl = pbCls_list[-1]
            self.control = self._decoder.protobufToCarla(pbControl)

    @staticmethod
    def listen(weak_self):
        self = weak_self()
        world = self.ego_vehicle.get_world()
        while True:
            world.wait_for_tick()
            if self.control is not None:
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

    player = None
    while player is None:
        print("Waiting for the ego vehicle...")
        time.sleep(1)
        possible_vehicles = sim_world.get_actors().filter('vehicle.*')
        for vehicle in possible_vehicles:
            if vehicle.attributes['role_name'] == ego_name:
                print("Ego vehicle found")
                player = vehicle
                break

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

    signal.pause()
