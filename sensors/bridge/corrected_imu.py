import math
import carla
from sensors.base_sensor import Sensor
from modules.localization.proto.imu_pb2 import CorrectedImu
# from modules.localization.proto.pose_pb2 import Pose
from sensors.bridge.carla_sensors import IMUSensor

class CorrectedImu(Sensor):
    """
        Carla(UE) X -- East, Y -- South, Z -- Up
        Position of the vehicle reference point (VRP) in the map reference frame.
        The VRP is the center of rear axle. apollo.common.PointENU position
        
    """
    _apollo_channel = '/apollo/sensor/gnss/corrected_imu'
    _apollo_msgType = 'apollo.localization.CorrectedImu'
    _apollo_pbCls = CorrectedImu
    _imu_sensor = None

    def __init__(self, ego_vehicle: carla.Vehicle, imu_sensor: IMUSensor) -> None:
        super().__init__(ego_vehicle)
        self._imu_sensor = imu_sensor        

    def update(self):
        transform = self.ego_vehicle.get_transform()
       
        self._pbCls.imu.linear_acceleration.x = self._imu_sensor.accelerometer[0]
        self._pbCls.imu.linear_acceleration.y = -self._imu_sensor.accelerometer[1]
        self._pbCls.imu.linear_acceleration.z = self._imu_sensor.accelerometer[2]

        self._pbCls.imu.angular_velocity.x = self._imu_sensor.gyroscope[1]
        self._pbCls.imu.angular_velocity.y = self._imu_sensor.gyroscope[0]
        self._pbCls.imu.angular_velocity.z = -self._imu_sensor.gyroscope[2]

        self._pbCls.imu.heading = -transform.rotation.yaw - 90

        self._pbCls.imu.euler_angles.x = math.radians(transform.rotation.pitch)
        self._pbCls.imu.euler_angles.y = math.radians(transform.rotation.roll)
        self._pbCls.imu.euler_angles.z = math.radians(-transform.rotation.yaw - 90)

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
