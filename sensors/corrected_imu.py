import math
import carla
import logging
from sensors.base_sensor import Sensor
from modules.localization.proto.imu_pb2 import CorrectedImu
from sensors.carla_sensors import IMUSensor, get_IMUSensor


class CorrectedImu(Sensor):
    """
        Carla(UE) X -- East, Y -- South, Z -- Up
        Position of the vehicle reference point (VRP) in the map reference frame.
        The VRP is the center of rear axle. apollo.common.PointENU position
    """
    _apollo_channel = '/apollo/sensor/gnss/corrected_imu'
    _apollo_msgType = 'apollo.localization.CorrectedImu'
    _apollo_pbCls = CorrectedImu

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: IMUSensor,
            freq: float = -1.,
            name: str = None) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                carla_sensor=carla_sensor,
                freq=freq,
                name=name)

    def update(self):
        transform = self.ego_vehicle.get_transform()

        self._pbCls.imu.linear_acceleration.x = self.carla_sensor.accelerometer[1]
        self._pbCls.imu.linear_acceleration.y = self.carla_sensor.accelerometer[0]
        self._pbCls.imu.linear_acceleration.z = self.carla_sensor.accelerometer[2] - 9.81

        self._pbCls.imu.angular_velocity.x = self.carla_sensor.gyroscope[1]
        self._pbCls.imu.angular_velocity.y = self.carla_sensor.gyroscope[0]
        self._pbCls.imu.angular_velocity.z = self.carla_sensor.gyroscope[2]

        self._pbCls.imu.heading = -transform.rotation.yaw - 90

        self._pbCls.imu.euler_angles.x = math.radians(transform.rotation.pitch)
        self._pbCls.imu.euler_angles.y = math.radians(transform.rotation.roll)
        self._pbCls.imu.euler_angles.z = math.radians(-transform.rotation.yaw - 90)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_CorrectedImu(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
    except KeyError as err:
        logging.error(err)
        raise ValueError
    imu_sensor = get_IMUSensor(ego_vehicle)
    return CorrectedImu(
        ego_vehicle, imu_sensor, freq, name)
