import math
import carla
import transforms3d
from sensors.base_sensor import Sensor
from sensors.carla_sensors import GnssSensor
from modules.localization.proto.gps_pb2 import Gps
from sensors.carla_sensors import GnssSensor, get_GnssSensor


class Odometry(Sensor):
    """
        Carla(UE) X -- East, Y -- South, Z -- Up
        Position of the vehicle reference point (VRP) in the map reference frame.
        The VRP is the center of rear axle. apollo.common.PointENU position
    """
    _apollo_channel = '/apollo/sensor/gnss/odometry'
    _apollo_msgType = 'apollo.localization.Gps'
    _apollo_pbCls = Gps

    def __init__(
            self,
            ego_vehicle: carla.Vehicle,
            carla_sensor: GnssSensor,
            freq: float = -1.,
            name: str = None,
            x_offset: float = 0.,
            y_offset: float = 0.) -> None:
        super().__init__(
                ego_vehicle=ego_vehicle,
                carla_sensor=carla_sensor,
                freq=freq,
                name=name)
        self.x_offset = x_offset
        self.y_offset = y_offset

    def update(self):
        transform = self.carla_sensor.transform
        linear_vel = self.ego_vehicle.get_velocity()

        self._pbCls.localization.position.x = transform.location.x + self.x_offset
        self._pbCls.localization.position.y = -transform.location.y + self.y_offset
        self._pbCls.localization.position.z = transform.location.z

        self._pbCls.localization.linear_velocity.x = linear_vel.x
        self._pbCls.localization.linear_velocity.y = -linear_vel.y
        self._pbCls.localization.linear_velocity.z = linear_vel.z

        rot_quat = transforms3d.euler.euler2quat(math.radians(self.ego_vehicle.get_transform().rotation.pitch),
                                                math.radians(self.ego_vehicle.get_transform().rotation.roll),
                                                math.radians(-self.ego_vehicle.get_transform().rotation.yaw - 90), 'rxyz')

        self._pbCls.localization.orientation.qw = rot_quat[0]
        self._pbCls.localization.orientation.qx = rot_quat[1]
        self._pbCls.localization.orientation.qy = rot_quat[2]
        self._pbCls.localization.orientation.qz = rot_quat[3]

        self._pbCls.localization.heading = -transform.rotation.yaw - 90

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


def get_Odometry(
        ego_vehicle: carla.Vehicle,
        config: dict):
    try:
        name = config['name']
        freq = config['frequency']
        x_offset = config['x_offset']
        y_offset = config['y_offset']
    except KeyError as err:
        print(err)
        raise ValueError
    gnss_sensor = get_GnssSensor(ego_vehicle)
    return Odometry(
        ego_vehicle, gnss_sensor, freq, name, x_offset, y_offset)
