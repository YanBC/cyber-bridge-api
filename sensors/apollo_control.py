import threading
import weakref
import carla
import logging
from cyber_bridge.cyber_bridge_client import CyberBridgeClient
from cyber_bridge.apollo_control_decoder import ApolloControlDecoder
from modules.control.proto.control_cmd_pb2 import ControlCommand
from utils import get_vehicle_by_role_name

import multiprocessing
import time
import enum


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
        self.timeout_rcv_cmd = 2
        self.lock = threading.Lock()
        self.control_is_ready = False
        self.cond = threading.Condition()

    @staticmethod
    def background(weak_self, stop_event: threading.Event):
        self = weak_self()
        lastRcv = time.time()
        while True:
            # self.bridge.recv_pb_messages is a blocking operation with timeout of 0.1s
            pbCls_list = self.bridge.recv_pb_messages()

            if stop_event.is_set():
                logging.info("stop event is set, thread background exiting ...")
                break

            if len(pbCls_list) == 0 and time.time() - lastRcv > self.timeout_rcv_cmd:
                logging.error(f"long time no message from apollo, exiting ..")
                break

            if len(pbCls_list) != 0:
                lastRcv = time.time()
                pbControl = pbCls_list[-1]
                with self.cond:
                    self.control_is_ready = True
                    self.control = self._decoder.protobufToCarla(pbControl)
                    self.cond.notify()
        if not stop_event.is_set():
            stop_event.set()

    @staticmethod
    def apply_control(weak_self, stop_event: threading.Event):
        self = weak_self()
        while True:
            with self.cond:
                while (not self.control_is_ready) and (not stop_event.is_set()):
                    self.cond.wait(0.1)

                if stop_event.is_set():
                    logging.info("stop event is set, thread apply_control exiting ...")
                    break

                if self.control_is_ready:
                    try:
                        self.ego_vehicle.apply_control(self.control)
                        self.control_is_ready = False
                    except Exception as e:
                        logging.error(f"fail to apply control {self.control}, {e}")
                        break
        if not stop_event.is_set():
            stop_event.set()


class ApolloControlArgs:
    def __init__(
                self,
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int) -> None:
        self.ego_name = ego_name
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.apollo_host = apollo_host
        self.apollo_port = apollo_port


class ApolloControlError(enum.Enum):
    SUCCESS = 0
    NETWORK_ERROR  = 2000
    NETWORK_ERROR_CARLA = 2001
    NETWORK_ERROR_APOLLO = 2002
    UNKNOWN_ERROR = 9999
    USER_INTERRUPT = 5001


class ApolloControlResults:
    def __init__(
            self, err_code=ApolloControlError.SUCCESS) -> None:
        self.err_code = err_code


def listen_and_apply_control(
                args: ApolloControlArgs,
                stop_event: multiprocessing.Event,
                output_queue: multiprocessing.Queue):
    ego_name = args.ego_name
    carla_host = args.carla_host
    carla_port = args.carla_port
    apollo_host = args.apollo_host
    apollo_port = args.apollo_port

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()
    error_code = ApolloControlError.SUCCESS

    player, _ = get_vehicle_by_role_name(
            stop_event, __name__, sim_world, ego_name)
    if stop_event.is_set():
        return

    sensor = ApolloControl(player, apollo_host, apollo_port)

    weak_self = weakref.ref(sensor)
    t1_stop_event = threading.Event()
    t1 = threading.Thread(
            target=ApolloControl.background,
            args=(weak_self, t1_stop_event),
            daemon=False)

    t2_stop_event = threading.Event()
    t2 = threading.Thread(
        target=ApolloControl.apply_control,
        args=(weak_self, t2_stop_event),
        daemon=False)
    t1.start()
    t2.start()

    error_code = ApolloControlError.SUCCESS
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
            if t1_stop_event.is_set():
                error_code = ApolloControlError.NETWORK_ERROR_APOLLO
                t2_stop_event.set()
                break
            if t2_stop_event.is_set():
                error_code = ApolloControlError.NETWORK_ERROR_CARLA
                t1_stop_event.set()
                break
    except Exception as e:
        if isinstance(e, KeyboardInterrupt):
            error_code = ApolloControlError.USER_INTERRUPT
        else:
            error_code = ApolloControlError.UNKNOWN_ERROR
    finally:
        if not stop_event.is_set():
            stop_event.set()
        if not t1_stop_event.is_set():
            t1_stop_event.set()
        if not t2_stop_event.is_set():
            t2_stop_event.set()
        t1.join()
        t2.join()
        result = ApolloControlResults(error_code)
        output_queue.put(result)
