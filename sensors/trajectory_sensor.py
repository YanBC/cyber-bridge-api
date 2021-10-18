import math
import numpy as np
import carla
from modules.canbus.proto.chassis_pb2 import Chassis
from sensors.base_sensor import Sensor
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.common.proto.pnc_point_pb2 import PathPoint, TrajectoryPoint
from agents.navigation.behavior_agent import BehaviorAgent


def stdvec(x):
    """ stdvec returns a 1D numpy array, given a list or 2d numpy array.
    """
    x = np.asarray(x)
    x = x.reshape(len(x),)
    return x


def smooth(x, n):
    """ smooth(x,n): smooths x over a moving window of size n. """
    x = stdvec(x)
    return np.convolve(x, np.ones((n,))/n, mode='full')


def curvature(x, y, n=False):
    """ Calculate the curvature for x, y coords.
    """
    # Make sure the format is correct.
    x = stdvec(x)
    y = stdvec(y)
    # Calculate first derivative.
    d1 = np.sqrt(np.diff(y)**2 + np.diff(x)**2)
    dx = np.diff(x)/(d1 + 1e-9)
    dy = np.diff(y)/(d1 + 1e-9)
    d1x = (dx[0:-1] + dx[1:])/2
    d1y = (dy[0:-1] + dy[1:])/2
    # Calculate second derivative.
    d2 = (d1[0:-1] + d1[1:])/2
    d2x = np.diff(dx)/(d2 + 1e-9)
    d2y = np.diff(dy)/(d2 + 1e-9)
    # Smooth the derivatives (optional).
    if n:
        d1x = smooth(d1x, n)
        d1y = smooth(d1y, n)
        d2x = smooth(d2x, n)
        d2y = smooth(d2y, n)
    # Calculate curvature.
    return (d1x*d2y - d1y*d2x)/((d1x**2 + d1y**2)**(3/2))


class TrajectorySensor(Sensor):
    _apollo_channel = '/apollo/planning'
    _apollo_msgType = 'apollo.planning.ADCTrajectory'
    _apollo_pbCls = ADCTrajectory

    def __init__(self, ego_vehicle: carla.Vehicle) -> None:
        super().__init__(ego_vehicle)
        self.agent = BehaviorAgent(ego_vehicle)
        destination = carla.Location(x=-50, y=-195)
        initialposi = self.ego_vehicle.get_location()
        self.agent.set_destination(initialposi, destination)

    def update(self):
        self.agent.update_information()
        incomming_waypoints = \
            self.agent._local_planner.waypoints_queue

        tt = []
        for wp in incomming_waypoints:
            # x, y, z, theta, kappa
            transform = wp[0].transform
            location = transform.location
            rotation = transform.rotation
            tt.append([
                        location.x,
                        -location.y,
                        location.z,
                        math.radians(rotation.yaw),
                        0])

        x = [t[0] for t in tt]
        y = [t[1] for t in tt]
        kappa = curvature(x, y)

        trajectory = ADCTrajectory()
        for idx in range(len(tt)):
            wp = tt[idx]
            if idx == 0 or idx == len(tt) - 1:
                k = 0
            else:
                k = kappa[idx-1]
            path_point = PathPoint(
                x=wp[0], y=wp[1], z=wp[2], theta=wp[3], kappa=k)
            trajectory_point = TrajectoryPoint(
                path_point=path_point, v=10)
            trajectory.trajectory_point.append(trajectory_point)

        self._pbCls.CopyFrom(trajectory)
        self._pbCls.gear = Chassis.GearPosition.GEAR_DRIVE
        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True


if __name__ == '__main__':
    x = [-74.50386810302734, -74.5161361694336, -74.52841186523438, -74.54068756103516, -74.5529556274414, -74.56523132324219, -74.57749938964844, -74.58977508544922, -74.60205078125, -74.61431884765625, -74.62659454345703, -74.63886260986328, -74.65113830566406, -74.66341400146484, -74.6756820678711, -74.68795776367188, -74.70571899414062, -74.7057113647461, -74.71797943115234, -74.73011016845703, -74.7188491821289, -74.66751098632812, -74.47669982910156, -74.47669982910156, -74.31390380859375, -74.11106872558594, -73.86859130859375, -73.61189270019531, -73.35519409179688, -72.99337005615234, -72.08746337890625, -70.61605834960938, -68.61923217773438, -66.15135955810547, -63.279624938964844, -60.082244873046875, -56.64625930786133]

    y = [-50.937400817871094, -55.43738555908203, -59.93737030029297, -64.4373550415039, -68.93733978271484, -73.43732452392578, -77.93730163574219, -82.43728637695312, -86.93727111816406, -91.43724822998047, -95.9372329711914, -100.43721771240234, -104.93720245361328, -109.43718719482422, -113.93717193603516, -118.43714904785156, -124.94712829589844, -124.94713592529297, -129.44711303710938, -133.94313049316406, -138.39584350585938, -142.84828186035156, -150.7816925048828, -150.7816925048828, -155.2314453125, -159.67958068847656, -164.1361846923828, -168.6288604736328, -173.1215362548828, -177.0105438232422, -180.5242156982422, -183.841064453125, -186.8707733154297, -189.53086853027344, -191.7489013671875, -193.46449279785156, -194.63092041015625]

    kappa = curvature(x, y)
