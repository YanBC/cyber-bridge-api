from carla import VehicleControl
from google.protobuf.message import Message

from decoders.base_decoder import BaseDecoder

from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.canbus.proto.chassis_pb2 import Chassis


class ApolloControlDecoder(BaseDecoder):
    def __init__(self, pbCls, channel:str, msgType:str) -> None:
        super().__init__(pbCls, channel, msgType)

    def protobufToCarla(self, pbMsg:Message) -> VehicleControl:
        pbCtrl = pbMsg
        carlaCtrl = VehicleControl()
        carlaCtrl.throttle = pbCtrl.throttle / 100
        carlaCtrl.steer = pbCtrl.steering_target / 100
        carlaCtrl.brake = pbCtrl.brake / 100
        carlaCtrl.hand_brake = pbCtrl.parking_brake
        carlaCtrl.reverse = pbCtrl.gear_location == Chassis.GearPosition.GEAR_REVERSE

        return carlaCtrl

