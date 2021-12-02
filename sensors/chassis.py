import math
import carla
import datetime
from sensors.base_sensor import Sensor
from modules.canbus.proto.chassis_pb2 import FIX_3D, Chassis
from sensors.carla_sensors import GnssSensor, get_GnssSensor


class ChassisAlter(Sensor):
    _apollo_channel = '/apollo/canbus/chassis'
    _apollo_msgType = 'apollo.canbus.Chassis'
    _apollo_pbCls = Chassis

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: GnssSensor,
            freq: float = -1.,
            name: str = None) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                carla_sensor=carla_sensor,
                freq=freq,
                name=name)

    def update(self):
        v = self.ego_vehicle.get_velocity()
        mps = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        control = self.ego_vehicle.get_control()

        self._pbCls.engine_started = True
        self._pbCls.speed_mps = mps
        self._pbCls.throttle_percentage = control.throttle * 100.0
        self._pbCls.brake_percentage = control.brake * 100.0
        coefficient = 1 # 70 / 29.375
        self._pbCls.steering_percentage = -1 * coefficient* control.steer * 100.0
        self._pbCls.parking_brake = control.hand_brake
        self._pbCls.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE

        self._pbCls.engine_started = True
        self._pbCls.gear_location = Chassis.GearPosition.GEAR_DRIVE

        self._pbCls.chassis_gps.latitude = self.carla_sensor.lat
        self._pbCls.chassis_gps.longitude = self.carla_sensor.lon
        now = datetime.datetime.now()
        self._pbCls.chassis_gps.year = now.year
        self._pbCls.chassis_gps.month = now.month
        self._pbCls.chassis_gps.day = now.day
        self._pbCls.chassis_gps.hours = now.hour
        self._pbCls.chassis_gps.minutes = now.minute
        self._pbCls.chassis_gps.seconds = now.second
        # self._pbCls.chassis_gps.hdop = 0.0100000000
        # self._pbCls.chassis_gps.vdop = 0.0100000000
        self._pbCls.chassis_gps.quality = FIX_3D
        self._pbCls.chassis_gps.heading = -self.ego_vehicle.get_transform().rotation.yaw - 90

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_ChassisAlter(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        print(err)
        raise ValueError
    gnss_sensor = get_GnssSensor(ego_vehicle)
    return ChassisAlter(
        ego_vehicle, gnss_sensor, freq, name)
