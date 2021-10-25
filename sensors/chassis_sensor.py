import math
import carla
from sensors.base_sensor import Sensor
from modules.canbus.proto.chassis_pb2 import Chassis


class ChassisSensor(Sensor):
    _apollo_channel = '/apollo/canbus/chassis'
    _apollo_msgType = 'apollo.canbus.Chassis'
    _apollo_pbCls = Chassis

    def __init__(self, ego_vehicle: carla.Vehicle) -> None:
        super().__init__(ego_vehicle)

    def update(self):
        v = self.ego_vehicle.get_velocity()
        mps = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        control = self.ego_vehicle.get_control()

        self._pbCls.engine_started = True
        self._pbCls.speed_mps = mps
        self._pbCls.throttle_percentage = control.throttle * 100.0
        self._pbCls.brake_percentage = control.brake * 100.0
        self._pbCls.steering_percentage = -1 * control.steer * 100.0
        self._pbCls.parking_brake = control.hand_brake
        self._pbCls.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE

        self._pbCls.engine_started = True
        self._pbCls.gear_location = Chassis.GearPosition.GEAR_DRIVE
        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
