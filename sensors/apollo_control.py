from datetime import datetime
import threading
import weakref
import multiprocessing
import carla
from cyber_bridge_client import CyberBridgeClient
from modules.control.proto.control_cmd_pb2 import ControlCommand
from decoders.apollo_control_decoder import ApolloControlDecoder
from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)

def emergency_stop():
    control = carla.VehicleControl()
    control.steer = 0.0
    control.throttle = 0.0
    control.brake = 1.0
    control.hand_brake = False
    return control


def previous_control(ego):
    return ego.get_control()


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
    def background(
            weak_self,
            sensors_ready: multiprocessing.Event,
            control_ready: threading.Event):
        self = weak_self()
        world = self.ego_vehicle.get_world()
        actor_type = self.ego_vehicle.type_id
        while True:
            if not is_actor_exist(world, actor_type=actor_type):
                break
            pbCls_list = self.bridge.recv_pb_messages()

            # if sensors_ready.is_set():
            #     if len(pbCls_list) == 0:
            #         # no control signal received
            #         # apply emergency stop
            #         print(f"{__name__}: no control cmd received, applying mergency stop")
            #         self.control = emergency_stop()
            #     else:
            #         pbControl = pbCls_list[-1]
            #         self.control = self._decoder.protobufToCarla(pbControl)
            #     sensors_ready.clear()
            #     control_ready.set()

            if len(pbCls_list) == 0:
                # no control signal received
                # apply emergency stop
                # now_time = datetime.now()
                # print(f"{__name__}[{now_time}]: no control cmd received, applying mergency stop")
                self.control = emergency_stop()
                # self.control = None
            else:
                pbControl = pbCls_list[-1]
                self.control = self._decoder.protobufToCarla(pbControl)

    @staticmethod
    def listen(
            weak_self,
            control_ready: threading.Event,
            ready_to_tick: multiprocessing.Event):
        self = weak_self()
        world = self.ego_vehicle.get_world()
        actor_type = self.ego_vehicle.type_id

        while True:
            if not is_actor_exist(world, actor_type=actor_type):
                break
            if self.control is None:
                continue

            # # world.wait_for_tick()
            # control_ready.wait()
            # self.ego_vehicle.apply_control(self.control)
            # control_ready.clear()
            # ready_to_tick.set()

            world.wait_for_tick()
            self.ego_vehicle.apply_control(self.control)


def listen_and_apply_control(
                sensors_ready: multiprocessing.Event,
                ready_to_tick: multiprocessing.Event,
                ego_name: str,
                carla_host: str,
                carla_port: int,
                apollo_host: str,
                apollo_port: int):

    def _set_ego_vehicle_physics(vehicle):
        # Create Wheels Physics Control
        front_left_wheel  = carla.WheelPhysicsControl(radius=33.5)
        front_right_wheel = carla.WheelPhysicsControl(radius=33.5)
        rear_left_wheel   = carla.WheelPhysicsControl(radius=33.5)
        rear_right_wheel  = carla.WheelPhysicsControl(radius=33.5)
        physics_control = vehicle.get_physics_control()

        print("length=%f, width=%f, height=%f " % (vehicle.bounding_box.extent.x * 2, vehicle.bounding_box.extent.y * 2, vehicle.bounding_box.extent.z * 2))
        print("rpm=%f, moi=%f, damping=%f, use_gear_autobox=%d, gear_switch_time=%f,\
             clutch_stength=%f, mass=%f, drag_coefficient=%f, ratio= %f" % ( physics_control.max_rpm,\
        physics_control.moi,\
        physics_control.damping_rate_full_throttle,\
        physics_control.use_gear_autobox,\
        physics_control.gear_switch_time,\
        physics_control.clutch_strength,\
        physics_control.mass,\
        physics_control.drag_coefficient,\
        physics_control.final_ratio ))
        print("center x = %f, center y=%f, center z=%f" % 
            (physics_control.center_of_mass.x, physics_control.center_of_mass.y, physics_control.center_of_mass.z))

        print("wheel0 = %f, wheel1 y=%f, wheel2=%f, wheel3 z=%f" % 
            (physics_control.wheels[0].max_steer_angle, physics_control.wheels[1].max_steer_angle, physics_control.wheels[2].max_steer_angle, physics_control.wheels[3].max_steer_angle))

        print("radius w0 = %f, w1 y=%f, w1=%f, w3 z=%f" % 
            (physics_control.wheels[0].radius , physics_control.wheels[1].radius , physics_control.wheels[2].radius, physics_control.wheels[3].radius))
        wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]    
        physics_control.wheels = wheels
        vehicle.apply_physics_control(physics_control)
        # print("wheel0 = %f, wheel1 y=%f, wheel2=%f, wheel3 z=%f" % \
        #       (physics_control.wheels[0].max_steer_angle, physics_control.wheels[1].max_steer_angle, physics_control.wheels[2].max_steer_angle, physics_control.wheels[3].max_steer_angle))

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, _ = get_vehicle_by_role_name(__name__, sim_world, ego_name)
    _set_ego_vehicle_physics(player)
    
    sensor = ApolloControl(player, apollo_host, apollo_port)

    control_ready = threading.Event()
    weak_self = weakref.ref(sensor)
    t1 = threading.Thread(
            target=ApolloControl.background,
            args=(weak_self, sensors_ready, control_ready),
            daemon=False)
    t2 = threading.Thread(
        target=ApolloControl.listen,
        args=(weak_self, control_ready, ready_to_tick),
        daemon=False)
    t1.start()
    t2.start()

    t1.join()
    t2.join()