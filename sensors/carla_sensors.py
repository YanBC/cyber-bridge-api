'''
Carla native sensors support
- All native sensors are implemented as singletons
'''
import carla
import weakref


class CarlaSensor:
    def destroy(self):
        raise NotImplementedError


# ===================================
# -- GnssSensor ---------------------
# ===================================
rear_to_center_in_x = -1.4224


class GnssSensor(CarlaSensor):
    def __init__(self, parent_actor, location=None):
        if location is None:
            location = carla.Location(x=rear_to_center_in_x)

        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.transform = None
        self.timestamp = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(location), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.transform = event.transform
        self.timestamp = event.timestamp

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


# ===================================
# -- IMUSensor ----------------------
# ===================================
class IMUSensor(CarlaSensor):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(carla.Location(x=rear_to_center_in_x)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], sensor_data.gyroscope.x)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.y)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.z)))
        self.compass = sensor_data.compass

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


# ===================================
# -- singletons ---------------------
# ===================================
_gnss_sensor = None
_imu_sensor = None


def get_GnssSensor(player: carla.Vehicle):
    global _gnss_sensor
    global rear_to_center_in_x

    location = carla.Location(x=rear_to_center_in_x)
    if _gnss_sensor is None:
        _gnss_sensor = GnssSensor(player, location)
    return _gnss_sensor


def get_IMUSensor(player: carla.Vehicle):
    global _imu_sensor
    if _imu_sensor is None:
        _imu_sensor = IMUSensor(player)
    return _imu_sensor
