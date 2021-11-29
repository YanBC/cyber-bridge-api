import math
import carla
import transforms3d
from sensors.base_sensor import Sensor
from sensors.carla_sensors import GnssSensor
from modules.localization.proto.gps_pb2 import Gps
# from modules.localization.proto.pose_pb2 import Pose

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
            gnss_sensor:GnssSensor,
            x_offset: float = 0.,
            y_offset: float = 0.) -> None:
        super().__init__(ego_vehicle)
        self._gnss_sensor = gnss_sensor
        self.x_offset = x_offset
        self.y_offset = y_offset

    def update(self):
        transform = self._gnss_sensor.transform
        linear_vel = self.ego_vehicle.get_velocity()

        self._pbCls.localization.position.x = transform.location.x + self.x_offset
        self._pbCls.localization.position.y = -transform.location.y + self.y_offset
        self._pbCls.localization.position.z = transform.location.z

        # print("gnss location x=%f, y=%f, z=%f" % (self._gnss_sensor.transform.location.x,
        #                                     self._gnss_sensor.transform.location.y,
        #                                     self._gnss_sensor.transform.location.z))
        # print("vehicle location x=%f, y=%f, z=%f" % (transform.location.x,
        #                                              transform.location.y,
        #                                              transform.location.z))

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
