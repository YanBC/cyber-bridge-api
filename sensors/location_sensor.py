import math
import carla
from sensors.base_sensor import Sensor
from modules.localization.proto.localization_pb2 import LocalizationEstimate


class LocationSensor(Sensor):
    _apollo_channel = '/apollo/localization/pose'
    _apollo_msgType = 'apollo.localization.LocalizationEstimate'
    _apollo_pbCls = LocalizationEstimate

    def __init__(self, ego_vehicle: carla.Vehicle) -> None:
        super().__init__(ego_vehicle)

    def update(self, time_stamp):
        transform = self.ego_vehicle.get_transform()
        linear_vel = self.ego_vehicle.get_velocity()
        angular_vel = self.ego_vehicle.get_angular_velocity()
        accel = self.ego_vehicle.get_acceleration()

        self._pbCls.pose.position.x = transform.location.x
        self._pbCls.pose.position.y = -transform.location.y
        self._pbCls.pose.position.z = transform.location.z
        self._pbCls.pose.linear_velocity.x = linear_vel.x
        self._pbCls.pose.linear_velocity.y = linear_vel.y
        self._pbCls.pose.linear_velocity.z = linear_vel.z
        self._pbCls.pose.angular_velocity_vrf.x = angular_vel.x
        self._pbCls.pose.angular_velocity_vrf.y = angular_vel.y
        self._pbCls.pose.angular_velocity_vrf.z = angular_vel.z
        self._pbCls.pose.linear_acceleration_vrf.x = accel.x
        self._pbCls.pose.linear_acceleration_vrf.y = accel.y
        self._pbCls.pose.linear_acceleration_vrf.z = accel.z
        self._pbCls.pose.heading = math.radians(-transform.rotation.yaw)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
