#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides all atomic scenario behaviors that reflect
trigger conditions to either activate another behavior, or to stop
another behavior.

For example, such a condition could be "InTriggerRegion", which checks
that a given actor reached a certain region on the map, and then starts/stops
a behavior of this actor.

The atomics are implemented with py_trees and make use of the AtomicCondition
base class
"""

from __future__ import print_function

import operator
import py_trees
import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from srunner.scenariomanager.scenarioatomics.atomic_behaviors\
    import calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions\
    import AtomicCondition

import srunner.tools as sr_tools

EPSILON = 0.001


def _retrieve_traffic_landmark(map: carla.Map,
                               vehicle: carla.Vehicle,
                               distance: float = 30.):
    tmp_map = map
    tmp_vehicle = vehicle
    traffic_light_landmarks = []
    waypoint_ego_near = tmp_map.get_waypoint(
            tmp_vehicle.get_location(),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
    if waypoint_ego_near is not None:
        landmarks = waypoint_ego_near.get_landmarks(distance)
        for landmark in landmarks:
            if landmark.type == '1000001':
                if len(traffic_light_landmarks) == 0:
                    traffic_light_landmarks.append(landmark)
                elif len(traffic_light_landmarks) > 0 \
                        and landmark.id != traffic_light_landmarks[-1].id:
                    traffic_light_landmarks.append(landmark)
    else:
        print('Generate ego vehicle waypoint failed!')
    return traffic_light_landmarks


def set_traffic_light_state(map, actor, light_state, light_keep_time=60.0):
    traffic_light = None
    _map = map
    _actor = actor
    # traffic_light = self.ego_vehicles[0].get_traffic_light()
    traffic_light_lm = _retrieve_traffic_landmark(_map,
                                                  _actor,
                                                  100)

    for tl_lm in traffic_light_lm:
        traffic_light = CarlaDataProvider.get_world().get_traffic_light(tl_lm)
        break

    if traffic_light is not None:
        if traffic_light.state != light_state:
            traffic_light.set_state(light_state)
            print("traffic light should be {} now".format(light_state))

        if carla.TrafficLightState.Green == light_state:
            traffic_light.set_green_time(light_keep_time)
        else:
            traffic_light.set_red_time(light_keep_time)
    else:
        print("not found traffic light")


class TriggerInSameLane(AtomicCondition):

    """
    Atomic containing a comparison between an actor's lane id
    and another actor's one. The behavior returns SUCCESS when the
    lane id is equal is achieved

    Args:
        actor (carla.Actor): actor from which the velocity is taken
        other_actor (carla.Actor): The actor with the reference velocity
        speed (float): Difference of speed between the actors
        name (string): Name of the condition
        comparison_operator: Type "operator", used to compare
                             the two velocities
    """

    def __init__(self, actor, other_actor,
                 name="TriggerInSameLane"):
        """
        Setup the parameters
        """
        super(TriggerInSameLane, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._map = CarlaDataProvider.get_map()
        self._actor = actor
        self._other_actor = other_actor

    def update(self):
        """
        Gets the speed of the two actors and compares them according to
        the comparison operator

        returns:
            py_trees.common.Status.RUNNING when the comparison fails and
            py_trees.common.Status.SUCCESS when it succeeds
        """
        new_status = py_trees.common.Status.RUNNING

        actor_waypoint =\
            self._map.get_waypoint(self._actor.get_location(),
                                   lane_type=(carla.LaneType.Driving |
                                              carla.LaneType.Shoulder |
                                              carla.LaneType.Sidewalk))
        other_actor_waypoint =\
            self._map.get_waypoint(self._other_actor.get_location(),
                                   lane_type=(carla.LaneType.Driving |
                                              carla.LaneType.Shoulder |
                                              carla.LaneType.Sidewalk))

        if actor_waypoint.road_id == other_actor_waypoint.road_id \
           and actor_waypoint.lane_id == other_actor_waypoint.lane_id:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTimeToArrivalToLocationBasedOnMaxSpeed(AtomicCondition):

    """
    This class contains a check if a actor arrives within a given time
    at a given location.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - time: The behavior is successful, if TTA is less than _time_ in seconds
    - location: Location to be checked in this behavior

    The condition terminates with SUCCESS, when the actor can reach the target
    location within the given time
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, actor, time, location, comparison_operator=operator.lt,
                 name="TimeToArrival"):
        """
        Setup parameters
        """
        super(InTimeToArrivalToLocationBasedOnMaxSpeed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._time = time
        self._target_location = location
        self._comparison_operator = comparison_operator
        self._actor_max_speed = CarlaDataProvider.get_velocity(self._actor)

    def update(self):
        """
        Check if the actor can arrive at target_location within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self._actor)

        if current_location is None:
            return new_status

        distance = calculate_distance(current_location, self._target_location)

        # Refer to the max speed
        if self._actor_max_speed < CarlaDataProvider.get_velocity(self._actor):
            velocity = CarlaDataProvider.get_velocity(self._actor)
            self._actor_max_speed = velocity
        else:
            velocity = self._actor_max_speed

        # if velocity is too small, simply use a large time to arrival
        time_to_arrival = self._max_time_to_arrival
        if velocity > EPSILON:
            time_to_arrival = distance / velocity

        if self._comparison_operator(time_to_arrival, self._time):
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTimeToArrivalToVehicleWithLaneDiff(AtomicCondition):

    """
    This class contains a check if a actor arrives within a given time
    at another actor.

    if the two vehicle run in different lane, the distance will be projected.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - time: The behavior is successful, if TTA is less than _time_ in seconds
    - other_actor: Reference actor used in this behavior

    The condition terminates with SUCCESS, when the actor can reach the other
    vehicle within the given time
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, actor, other_actor, time, condition_freespace=False,
                 along_route=False, comparison_operator=operator.lt,
                 name="InTimeToArrivalToVehicleWithLaneDiff"):
        """
        Setup parameters
        """
        super(InTimeToArrivalToVehicleWithLaneDiff, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._map = CarlaDataProvider.get_map()
        self._actor = actor
        self._other_actor = other_actor
        self._time = time
        self._condition_freespace = condition_freespace
        self._along_route = along_route
        self._comparison_operator = comparison_operator

        if self._along_route:
            # Get the global route planner, used to calculate the route
            dao = GlobalRoutePlannerDAO(self._map, 0.5)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp
        else:
            self._grp = None

    def update(self):
        """
        Check if the ego vehicle can arrive at other actor within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self._actor)
        other_location = CarlaDataProvider.get_location(self._other_actor)

        # Get the bounding boxes
        if self._condition_freespace:
            if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
                actor_extent = self._actor.bounding_box.extent.x
            else:
                # Patch, as currently static objects have no bounding boxes
                actor_extent = 0

            if isinstance(self._other_actor, (carla.Vehicle, carla.Walker)):
                other_extent = self._other_actor.bounding_box.extent.x
            else:
                # Patch, as currently static objects have no bounding boxes
                other_extent = 0

        if current_location is None or other_location is None:
            return new_status

        current_velocity = CarlaDataProvider.get_velocity(self._actor)
        other_velocity = CarlaDataProvider.get_velocity(self._other_actor)

        if self._along_route:
            # Global planner needs a location at a driving lane
            current_location =\
                self._map.get_waypoint(current_location).transform.location
            other_location =\
                self._map.get_waypoint(other_location).transform.location

        distance =\
            calculate_distance(current_location, other_location, self._grp)

        actor_wp = self._map.get_waypoint(current_location)
        other_wp = self._map.get_waypoint(other_location)
        fv = self._actor.get_transform().rotation.get_forward_vector()
        # print("x[{}],y[{}],z[{}]".format(fv.x, fv.y, fv.z))

        if actor_wp is not None and other_wp is not None \
           and actor_wp.lane_id != other_wp.lane_id:
            diff_location = other_location - current_location
            distance = \
                fv.x * diff_location.x + fv.y * diff_location.y +\
                fv.z * diff_location.z

        # if velocity is too small, simply use a large time to arrival
        time_to_arrival = self._max_time_to_arrival

        if current_velocity > other_velocity:
            if self._condition_freespace:
                time_to_arrival = (distance - actor_extent - other_extent) /\
                                  (current_velocity - other_velocity)
            else:
                time_to_arrival = distance /\
                                  (current_velocity - other_velocity)

        if self._comparison_operator(time_to_arrival, self._time):
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTriggertoVehiclePassDistance(AtomicCondition):

    """
    comment
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, actor, other_actor, distance=10,
                 condition_freespace=False,
                 along_route=False, name="InTriggertoVehiclePassDistance"):
        """
        Setup parameters
        """
        super(InTriggertoVehiclePassDistance, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._map = CarlaDataProvider.get_map()
        self._actor = actor
        self._other_actor = other_actor
        self._distance = distance
        self._condition_freespace = condition_freespace
        self._along_route = along_route

        if self._along_route:
            # Get the global route planner, used to calculate the route
            dao = GlobalRoutePlannerDAO(self._map, 0.5)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp
        else:
            self._grp = None

    def update(self):
        """
        Check if the ego vehicle can arrive at other actor within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self._actor)
        other_location = CarlaDataProvider.get_location(self._other_actor)
        actor_extent = 0
        other_extent = 0

        # Get the bounding boxes
        if self._condition_freespace:
            if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
                actor_extent = self._actor.bounding_box.extent.x
            else:
                # Patch, as currently static objects have no bounding boxes
                actor_extent = 0

            if isinstance(self._other_actor, (carla.Vehicle, carla.Walker)):
                other_extent = self._other_actor.bounding_box.extent.x
            else:
                # Patch, as currently static objects have no bounding boxes
                other_extent = 0

        if current_location is None or other_location is None:
            return new_status

        if self._along_route:
            # Global planner needs a location at a driving lane
            current_location =\
                self._map.get_waypoint(current_location).transform.location
            other_location =\
                self._map.get_waypoint(other_location).transform.location

        distance =\
            calculate_distance(current_location, other_location, self._grp)

        actor_wp = self._map.get_waypoint(current_location)
        other_wp = self._map.get_waypoint(other_location)
        fv = self._actor.get_transform().rotation.get_forward_vector()
        # print("x[{}],y[{}],z[{}]".format(fv.x, fv.y, fv.z))

        if actor_wp is not None and other_wp is not None \
           and actor_wp.lane_id != other_wp.lane_id:
            diff_location = other_location - current_location
            project_distance =\
                fv.x * diff_location.x + fv.y * diff_location.y +\
                fv.z * diff_location.z
            if project_distance < 0 and distance >\
               (self._distance + actor_extent + other_extent):
                new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTriggertoVehicleArrive(AtomicCondition):

    """
    This class contains a check if a actor the other one, the distance between
    them is considered.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - time: The behavior is successful, if TTA is less than _time_ in seconds
    - other_actor: Reference actor used in this behavior

    The condition terminates with SUCCESS, when the actor can reach the other
    vehicle within the given time
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, actor, destination, condition_freespace=False,
                 name="InTriggertoVehiclePassDistance"):
        """
        Setup parameters
        """
        super(InTriggertoVehiclePassDistance, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._map = CarlaDataProvider.get_map()
        self._actor = actor
        self._destination = destination
        self._condition_freespace = condition_freespace

    def update(self):
        """
        Check if the ego vehicle can arrive at other actor within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self._actor)
        actor_extent_x = 0
        actor_extent_y = 0

        # Get the bounding boxes
        if self._condition_freespace:
            if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
                actor_extent_x = self._actor.bounding_box.extent.x
                actor_extent_y = self._actor.bounding_box.extent.y

        if current_location is None:
            return new_status

        lane_width = self._map.get_waypoint(current_location).lane_width
        if lane_width is None:
            return new_status

        project_location =\
            self._map.get_waypoint(current_location).transform.location
        location_diff_from_lane_center = current_location - project_location
        rv = self._actor.get_transform().rotation.get_right_vector()
        distane_to_lane_center =\
            location_diff_from_lane_center.x * rv.x +\
            location_diff_from_lane_center.y * rv.y +\
            location_diff_from_lane_center.z * rv.z

        location_diff_from_destination = self._destination - current_location
        fv = self._actor.get_transform().rotation.get_forward_vector()
        distane_to_destination = \
            location_diff_from_destination.x * fv.x +\
            location_diff_from_destination.y * fv.y +\
            location_diff_from_destination.z * fv.z

        dis_from_actor_right_to_lane_line =\
            lane_width/2 - distane_to_lane_center - actor_extent_y
        dis_from_actor_head_to_des = distane_to_destination - actor_extent_x

        if dis_from_actor_right_to_lane_line <= 0.3\
           and dis_from_actor_head_to_des <= 10:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InArrivalToLocation(AtomicCondition):

    """
    This class contains a check if a actor arrives at a given location.

    """

    def __init__(self, actor, location, name="InArrivalToLocation"):
        """
        Setup parameters
        """
        super(InArrivalToLocation, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._target_location = location

    def update(self):
        """
        Check if the actor can arrive at target_location within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self._actor)
        if current_location is None:
            return new_status

        if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
            actor_extent_x = self._actor.bounding_box.extent.x
        else:
            # Patch, as currently static objects have no bounding boxes
            actor_extent_x = 0

        distance = calculate_distance(current_location, self._target_location)
        if actor_extent_x > EPSILON + distance:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTriggerDistanceToLocationSetTrafficLight(AtomicCondition):

    """
    This class contains the trigger (condition) for a distance to a fixed
    location of a scenario

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - target_location: Reference location (carla.location)
    - name: Name of the condition
    - distance: Trigger distance between the actor and the target location
                in meters

    The condition terminates with SUCCESS, when the actor reached the target
    distance to the given location
    """

    def __init__(self,
                 actor,
                 target_location,
                 distance,
                 comparison_operator=operator.lt,
                 name="InTriggerDistanceToLocationSetTrafficLight"):
        """
        Setup trigger distance
        """
        super(InTriggerDistanceToLocationSetTrafficLight, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target_location = target_location
        self._actor = actor
        self._map = CarlaDataProvider.get_map()
        self._distance = distance
        self._comparison_operator = comparison_operator

    def update(self):
        """
        Check if the actor is within trigger distance to the target location
        """

        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._actor)

        if location is None:
            return new_status

        distance = calculate_distance(location, self._target_location)
        # print("distance to destination={}".format(distance))
        if self._comparison_operator(distance, self._distance):
            set_traffic_light_state(self._map, self._actor,
                                    carla.TrafficLightState.Green, 0.1)
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTriggerDistancePassVehicle(AtomicCondition):

    """
    This class contains the trigger distance (condition) between to actors
    of a scenario

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - reference_actor: Reference CARLA actor
    - name: Name of the condition
    - distance: Trigger distance between the two actors in meters
    - distance_type: Specifies how distance should be calculated between
                     the two actors
    - freespace: if True distance is calculated between closest boundary
                 points else it will be from center-center
    - dx, dy, dz: distance to reference_location (location of reference_actor)

    The condition terminates with SUCCESS, when the actor reached the target
    distance to the other actor
    """

    def __init__(self, reference_actor, actor, distance,
                 comparison_operator=operator.gt,
                 distance_type="cartesianDistance",
                 freespace=False, name="InTriggerDistancePassVehicle"):
        """
        Setup trigger distance
        """
        super(InTriggerDistancePassVehicle, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._reference_actor = reference_actor
        self._actor = actor
        self._distance = distance
        self._distance_type = distance_type
        self._freespace = freespace
        self._comparison_operator = comparison_operator

        if distance_type == "longitudinal":
            dao =\
                GlobalRoutePlannerDAO(CarlaDataProvider.get_world().get_map(),
                                      1.0)
            self._global_rp = GlobalRoutePlanner(dao)
            self._global_rp.setup()
        else:
            self._global_rp = None

    def update(self):
        """
        Check if the ego vehicle is within trigger distance to other actor
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._actor)
        reference_location =\
            CarlaDataProvider.get_location(self._reference_actor)

        if location is None or reference_location is None:
            return new_status

        distance = sr_tools.scenario_helper.\
            get_distance_between_actors(self._actor,
                                        self._reference_actor,
                                        distance_type=self._distance_type,
                                        freespace=self._freespace,
                                        global_planner=self._global_rp)

        ref_vec = location - reference_location
        forward_vec =\
            self._reference_actor.get_transform().rotation.get_forward_vector()
        has_pass = forward_vec.x * ref_vec.x + forward_vec.y * ref_vec.y < 0
        # print("has pass = {}".format(has_pass))
        if self._comparison_operator(distance, self._distance) and has_pass:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status


class InTriggerVelocityDecelerate(AtomicCondition):

    """
    This class contains the trigger velocity decelerating (condition)
    between to actors of a scenario

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - reference_actor: Reference CARLA actor
    - name: Name of the condition
    - velocity_decelerate: velocity decelerate threshold

    The condition terminates with SUCCESS, when the actor reached the target
    distance to the other actor
    """

    def __init__(self, actor, velocity_decelerate,
                 comparison_operator=operator.gt,
                 name="InTriggerVelocityDecelerate"):
        """
        Setup trigger distance
        """
        super(InTriggerVelocityDecelerate, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._velocity_decelerate = velocity_decelerate
        self._comparison_operator = comparison_operator
        self._max_velocity = 0

    def update(self):
        """
        Check if the ego vehicle is within trigger distance to other actor
        """
        new_status = py_trees.common.Status.RUNNING

        cur_velocity = CarlaDataProvider.get_velocity(self._actor)
        if cur_velocity > self._max_velocity:
            self._max_velocity = cur_velocity

        if self._comparison_operator(self._max_velocity - cur_velocity,
                                     self._velocity_decelerate):
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__,
                                                   self.status, new_status))

        return new_status
