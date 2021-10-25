from carla import VehicleControl
from google.protobuf.message import Message

from encoders.base_encoder import BaseEncoder

from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.canbus.proto.chassis_pb2 import Chassis


class ApolloControlEncoder(BaseEncoder):
    def __init__(self, pbCls, channel:str, msgType:str) -> None:
        super().__init__(pbCls, channel, msgType)

    def carlaToProtobuf(self, carlaCtrl:VehicleControl) -> Message:
        # extract info
        throttle: float = carlaCtrl.throttle # 0 ~ 1
        steer: float = carlaCtrl.steer # -1 ~ 1
        brake: float = carlaCtrl.brake # 0 ~ 1
        hand_brake: bool = carlaCtrl.hand_brake
        reverse: bool = carlaCtrl.reverse
        # manual_gear_shift: bool = carlaCtrl.manual_gear_shift
        # gear: int = carlaCtrl.gear

        # convert info
        pbCtrl = ControlCommand()
        pbCtrl.throttle = throttle * 100
        pbCtrl.steering_target = -1 * steer * 100
        pbCtrl.brake = brake * 100
        pbCtrl.parking_brake = hand_brake
        if reverse:
            pbCtrl.gear_location = Chassis.GearPosition.GEAR_REVERSE
        else:
            pbCtrl.gear_location = Chassis.GearPosition.GEAR_DRIVE

        return pbCtrl
