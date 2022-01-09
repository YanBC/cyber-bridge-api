#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function
import logging

import math
import random
import py_trees
import carla
import operator

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors \
    import (ActorTransformSetter,
            ActorDestroy,
            AccelerateToVelocity,
            HandBrakeVehicle,
            KeepVelocity,
            StopVehicle,
            WaypointFollower,
            LaneChange,
            SyncArrival)
from srunner.scenariomanager.scenarioatomics.atomic_criteria \
    import (CollisionTest,
            KeepLaneTest)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions \
    import (InTriggerDistanceToLocationAlongRoute,
            InTimeToArrivalToLocation,
            DriveDistance,
            StandStill,
            TriggerVelocity,
            InTriggerDistanceToVehicle,
            InTriggerDistanceToNextIntersection,
            WaitUntilInFront,
            InTriggerRegion)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (get_location_in_distance_from_wp,
                                           get_waypoint_in_distance)

from scenarios_manager.scenarioatomics.atomic_trigger_conditions \
    import (set_traffic_light_state,
            InTimeToArrivalToLocationBasedOnMaxSpeed,
            InTimeToArrivalToVehicleWithLaneDiff,
            InArrivalToLocation,
            InTriggerDistanceToLocationSetTrafficLight,
            InTriggerDistancePassVehicle,
            InTriggerVelocityDecelerate)
from scenarios_manager.scenarioatomics.atomic_behaviors import (Fail)
from scenarios_manager.scenarioatomics.atomic_criteria \
    import (VehicleRunTest,
            InArriveRegionTest,
            MaxSpeedLimitTest,
            ArriveAtLocationTest,
            StandStillTimeTest,
            StandStillDistanceTest,
            NeverStopTest,
            VehicleTurnSignalTest,
            MoveOnRunWhenMeetObsTest)


class StationaryVehicleCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)

        self._destination_wp = self._wmap.get_waypoint(config.destination)
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryVehicleCrossing, self)\
            .__init__("StationaryVehicleCrossing",
                      ego_vehicles,
                      config,
                      world,
                      debug_mode,
                      criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _start_distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = \
            get_location_in_distance_from_wp(self._reference_waypoint,
                                             _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location,
                                         carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                     self.transform)
        static.set_simulate_physics(True)
        self.other_actors.append(static)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
            self.ego_vehicles[0].get_location()).lane_width
        lane_width = lane_width + (1.25 * lane_width)

        # leaf nodes
        just_dirive = \
            InArrivalToLocation(self.ego_vehicles[0],
                                self._destination_wp.transform.location)
        end_condition = \
            InTriggerDistancePassVehicle(self.ego_vehicles[0],
                                         self.other_actors[0],
                                         comparison_operator=operator.gt)

        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0],
                                                         self.transform))
        scenario_sequence.add_child(just_dirive)
        root.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class DynamicObjectCrossingWithStop(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        # config.trigger_points[0].location is about 'ego vehicle loaction
        # in config file'
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40  # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        # flag to select either pedestrian (False) or cyclist (True)
        self._adversary_type = adversary_type
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0
        self._ego_vehicle_speed_limit = 10
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(DynamicObjectCrossingWithStop, self).\
            __init__("DynamicObjectCrossingWithStop",
                     ego_vehicles,
                     config,
                     world,
                     debug_mode,
                     criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint,
                                                       _start_distance,
                                                       stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)),\
            orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        # self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            self._other_actor_target_velocity = 1.8  # 6.5km/h
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            self._other_actor_target_velocity = 4.5  # 16km/h
            first_vehicle = CarlaDataProvider.\
                request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of
        the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().\
            get_waypoint(transform.location)

        self.transform2 = carla.Transform(
            carla.Location(x_static, y_static,
                           spawn_point_wp.transform.location.z + 0.3),
            carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor(
            'static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        self._other_actor_stop_location, _ = \
            get_location_in_distance_from_wp(self._reference_waypoint,
                                             _start_distance,
                                             True)
        if self._other_actor_stop_location is None:
            raise ValueError("Not found the other actor")

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = \
                    self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform,
                                                      orientation_yaw)
                blocker = self._spawn_blocker(self.transform, orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to
        # the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z - 500),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z - 500),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = \
                InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                      self._ego_route,
                                                      self.transform.location,
                                                      dist_to_trigger)
        else:
            start_condition = \
                InTimeToArrivalToLocationBasedOnMaxSpeed(
                    self.ego_vehicles[0],
                    self._time_to_reach + 2.0,
                    self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")

        other_actor_drive_distance = \
            self.transform.location.distance(self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = \
            AccelerateToVelocity(self.other_actors[0],
                                 1.0,
                                 self._other_actor_target_velocity,
                                 name="walker crossing lane acc velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker crossing lane")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        actor_remove = ActorDestroy(self.other_actors[0],
                                    name="Destroying walker")
        static_remove = ActorDestroy(self.other_actors[1],
                                     name="Destroying Prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0],
                                       name="actor stand still")
        actor_continue_condition = \
            TriggerVelocity(self.ego_vehicles[0],
                            0.001,
                            comparison_operator=operator.lt,
                            name="walker continue condition")

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity")
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep stand")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self.transform,
                                 name='TransformSetterTS3walker'))
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[1], self.transform2,
                                 name='TransformSetterTS3coca', physics=False))
        scenario_sequence.add_child(
            HandBrakeVehicle(self.other_actors[0], True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(
            HandBrakeVehicle(self.other_actors[0], False))
        scenario_sequence.add_child(keep_velocity)
        # scenario_sequence.add_child(continue_drive)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(keep_stand)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(actor_remove)
        scenario_sequence.add_child(static_remove)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)

        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)
        keep_stand.add_child(actor_stand_still)
        keep_stand.add_child(actor_continue_condition)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class WalkerCrossingWithStop(BasicScenario):

    """
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road. it will stop
    unit the cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40  # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0
        self._ego_vehicle_speed_limit = 10
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(WalkerCrossingWithStop, self).\
            __init__("WalkerCrossingWithStop",
                     ego_vehicles,
                     config,
                     world,
                     debug_mode,
                     criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint,
                                                       _start_distance,
                                                       stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + \
            offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)),\
            orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        # self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            self._other_actor_target_velocity = 1.8  # 6.5km/h
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            self._other_actor_target_velocity = 4.5  # 16km/h
            first_vehicle = \
                CarlaDataProvider.request_new_actor(
                    'vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of
        the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = \
            self.ego_vehicles[0].get_world().get_map().\
            get_waypoint(transform.location)

        self.transform2 = \
            carla.Transform(
                carla.Location(x_static,
                               y_static,
                               spawn_point_wp.transform.location.z + 0.3),
                carla.Rotation(yaw=orientation_yaw + 180))

        static = \
            CarlaDataProvider.request_new_actor('static.prop.vendingmachine',
                                                self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        self._other_actor_stop_location, _ = \
            get_location_in_distance_from_wp(self._reference_waypoint,
                                             _start_distance,
                                             True)
        if self._other_actor_stop_location is None:
            raise ValueError("Not found the other actor")

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = \
                    self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = \
                    self._spawn_adversary(self.transform, orientation_yaw)
                blocker = self._spawn_blocker(self.transform, orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn

                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to
        # the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z + 1),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z - 500),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="WalkerCrossingWithStop")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = \
                InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                      self._ego_route,
                                                      self.transform.location,
                                                      dist_to_trigger)
        else:
            start_condition = \
                InTimeToArrivalToLocationBasedOnMaxSpeed(
                    self.ego_vehicles[0],
                    self._time_to_reach + 2.0,
                    self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")

        other_actor_drive_distance = \
            self.transform.location.distance(self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = \
            AccelerateToVelocity(self.other_actors[0],
                                 1.0,
                                 self._other_actor_target_velocity,
                                 name="walker crossing lane acc velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive for lane crossing")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0],
                                       name="walker stand still")
        actor_continue_condition = \
            TriggerVelocity(self.ego_vehicles[0],
                            0.001,
                            comparison_operator=operator.lt,
                            name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity")
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep stand")
        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0],
                                 self.transform,
                                 name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0],
                                                     True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0],
                                                     False))
        scenario_sequence.add_child(keep_velocity)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(keep_stand)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)
        keep_stand.add_child(actor_stand_still)
        keep_stand.add_child(actor_continue_condition)

        root.add_child(
            InTriggerDistancePassVehicle(self.ego_vehicles[0],
                                         self.other_actors[0],
                                         5,
                                         comparison_operator=operator.gt))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CyclistCrossingWithStop(BasicScenario):

    """
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road. it will stop unit
    the cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40
        # other vehicle parameters
        self._other_actor_target_velocity = 14/3.6
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0
        self._ego_vehicle_speed_limit = 10
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(CyclistCrossingWithStop, self).\
            __init__("CyclistCrossingWithStop",
                     ego_vehicles,
                     config,
                     world,
                     debug_mode,
                     criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = \
            get_location_in_distance_from_wp(waypoint,
                                             _start_distance,
                                             stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)),\
            orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):
        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            self._other_actor_target_velocity = 1.8
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            self._other_actor_target_velocity = 4.5
            first_vehicle = \
                CarlaDataProvider.request_new_actor(
                    'vehicle.diamondback.century',
                    transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of
        the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().\
            get_waypoint(transform.location)

        self.transform2 = carla.Transform(
            carla.Location(x_static, y_static,
                           spawn_point_wp.transform.location.z + 0.3),
            carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor(
            'static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit

        # _start_distance = 20
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        self._other_actor_stop_location, _ = \
            get_location_in_distance_from_wp(self._reference_waypoint,
                                             _start_distance, True)
        if self._other_actor_stop_location is None:
            raise ValueError("not found the other actor")

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = \
                    self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = \
                    self._spawn_adversary(self.transform, orientation_yaw)

                blocker = self._spawn_blocker(self.transform, orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to
        # the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z + 1),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z - 500),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="WalkerCrossingWithStop")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = \
                InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                      self._ego_route,
                                                      self.transform.location,
                                                      dist_to_trigger)
        else:
            start_condition = \
                InTimeToArrivalToLocationBasedOnMaxSpeed(
                    self.ego_vehicles[0],
                    self._time_to_reach + 2.0,
                    self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")

        other_actor_drive_distance = self.transform.location.distance(
            self._other_actor_stop_location)

        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = \
            AccelerateToVelocity(self.other_actors[0],
                                 1.0,
                                 self._other_actor_target_velocity,
                                 name="walker crossing lane accvelocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive for lane crossing")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")

        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0],
                                       name="walker stand still")
        actor_continue_condition = \
            TriggerVelocity(self.ego_vehicles[0],
                            0.001,
                            comparison_operator=operator.lt,
                            name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity")
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep stand")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self.transform,
                                 name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0],
                                                     True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0],
                                                     False))
        scenario_sequence.add_child(keep_velocity)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(keep_stand)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)
        keep_stand.add_child(actor_stand_still)
        keep_stand.add_child(actor_continue_condition)

        root.add_child(
            InTriggerDistancePassVehicle(self.ego_vehicles[0],
                                         self.other_actors[0],
                                         5, comparison_operator=operator.gt))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class WalkerCrossingWithPropAndStop(BasicScenario):

    """
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road. it will stop unit
    the cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40  # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0
        self._ego_vehicle_speed_limit = 10
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(WalkerCrossingWithPropAndStop, self).__init__(
            "WalkerCrossingWithPropAndStop",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint,
                                                       _start_distance,
                                                       stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)),\
            orientation_yaw

    def _calculate_prop_base_transform(self, _start_distance, waypoint):

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint,
                                                       _start_distance,
                                                       stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        orientation_yaw = waypoint.transform.rotation.yaw
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)),\
            orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            self._other_actor_target_velocity = 1.8
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            self._other_actor_target_velocity = 4.5
            first_vehicle = CarlaDataProvider.request_new_actor(
                'vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of
        the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = \
            self.ego_vehicles[0].get_world().get_map().get_waypoint(
                transform.location)

        self.transform2 = carla.Transform(
            carla.Location(x_static, y_static,
                           spawn_point_wp.transform.location.z + 0.3),
            carla.Rotation(yaw=orientation_yaw))

        static = CarlaDataProvider.request_new_actor(
            'vehicle.tesla.model3', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit

        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        self._other_actor_stop_location, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance, True)
        if self._other_actor_stop_location is None:
            raise ValueError("not found the other actor")

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = \
                    self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(
                    self.transform, orientation_yaw)

                blocker_base_transform, blocker_orientation_yaw = \
                    self._calculate_prop_base_transform(
                        _start_distance, waypoint)
                blocker = self._spawn_blocker(blocker_base_transform,
                                              blocker_orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to
        # the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z - 500),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z - 500),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = InTriggerDistanceToLocationAlongRoute(
                self.ego_vehicles[0],
                self._ego_route,
                self.transform.location,
                dist_to_trigger)
        else:
            start_condition = InTimeToArrivalToLocationBasedOnMaxSpeed(
                self.ego_vehicles[0],
                self._time_to_reach + 2.0,
                self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        other_actor_drive_distance = self.transform.location.distance(
            self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(
            self.other_actors[0],
            1.0,
            self._other_actor_target_velocity,
            name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive for lane crossing")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0],
                                       name="walker stand still")
        actor_continue_condition = \
            TriggerVelocity(self.ego_vehicles[0],
                            0.001,
                            comparison_operator=operator.lt,
                            name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep velocity")
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="keep stand")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(
                self.other_actors[1], self.transform2,
                name='TransformSetterTS3coca', physics=False))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self.transform,
                                 name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0],
                                                     False))
        scenario_sequence.add_child(keep_velocity)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(keep_stand)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)
        keep_stand.add_child(actor_stand_still)
        keep_stand.add_child(actor_continue_condition)

        root.add_child(
            InTriggerDistancePassVehicle(self.ego_vehicles[0],
                                         self.other_actors[0],
                                         5, comparison_operator=operator.gt))
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithLanechangeAndObstacle(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle,
    so that the leading vehicle will change lane then.

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 25
        self._second_actor_location = self._first_actor_location + 60
        self._first_actor_speed = 11  # about 40km/h
        self._second_actor_speed = 0
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None
        self._direction = 'left'

        super(FollowLeadingVehicleWithLanechangeAndObstacle,
              self).__init__("FollowLeadingVehicleWithLanechangeAndObstacle",
                             ego_vehicles,
                             config,
                             world,
                             debug_mode,
                             criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        if 'LEFT' in self.config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self.config.name.upper():
            self._direction = 'right'
        print("direction={}".format(self._direction))

        first_actor_waypoint, _ = get_waypoint_in_distance(
            self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _ = get_waypoint_in_distance(
            self._reference_waypoint, self._second_actor_location)

        first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z - 500),
            first_actor_waypoint.transform.rotation)
        self._first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z + 1),
            first_actor_waypoint.transform.rotation)

        yaw_1 = second_actor_waypoint.transform.rotation.yaw
        second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z - 500),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch,
                           yaw_1,
                           second_actor_waypoint.transform.rotation.roll))
        self._second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z + 1),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch,
                           yaw_1,
                           second_actor_waypoint.transform.rotation.roll))

        first_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_actor_transform)
        second_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', second_actor_transform)

        first_actor.set_simulate_physics(enabled=False)
        second_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(first_actor)
        self.other_actors.append(second_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario.After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region,then make other actor to drive towards obstacle.
        Once obstacle clears the road,make the other actor to drive towards the
        next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        stop_near_intersection = py_trees.composites.Parallel(
            "Waiting for end position near Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0],
                                                          10))
        stop_near_intersection.add_child(
            InTriggerDistanceToNextIntersection(self.other_actors[0], 20))

        driving_to_next_intersection.add_child(
            WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(
            InTriggerDistanceToVehicle(self.other_actors[1],
                                       self.other_actors[0],
                                       distance=30))

        endcondition = \
            py_trees.composites.Parallel(
                "Waiting for end position",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(
            self.ego_vehicles[0], self.other_actors[1]))
        endcondition.add_child(StandStill(
            self.ego_vehicles[0], name="FinalSpeed", duration=1))
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(
            self.other_actors[0], self._first_actor_transform))
        sequence.add_child(ActorTransformSetter(
            self.other_actors[1], self._second_actor_transform))
        sequence.add_child(AccelerateToVelocity(self.other_actors[0],
                                                1.0,
                                                self._first_actor_speed))
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(LaneChange(self.other_actors[0]))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithDecelerate(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading
    vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 14
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleWithDecelerate, self).__init__(
            "FollowLeadingVehicleWithDecelerate",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(
            self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario.After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make other actor to drive until reaching
        the next intersection. Finally, user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel(
            "Waiting for end position",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(
            InTriggerDistancePassVehicle(
                self.ego_vehicles[0],
                self.other_actors[0],
                comparison_operator=operator.gt, distance=5))
        endcondition.add_child(StandStill(
            self.ego_vehicles[0], name="FinalSpeed", duration=1))
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(
            self.other_actors[0], self._other_actor_transform))
        sequence.add_child(KeepVelocity(
            self.other_actors[0], 50/3.6, distance=30))
        sequence.add_child(StopVehicle(
            self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StationaryVehicleCrossingLane(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        self._destination_wp = self._wmap.get_waypoint(config.destination)

        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryVehicleCrossingLane, self).__init__(
            "StationaryVehicleCrossingLane",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _start_distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.8}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location,
                                         carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                     self.transform)
        static.set_simulate_physics(True)
        self.other_actors.append(static)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
            self.ego_vehicles[0].get_location()).lane_width
        lane_width = lane_width + (1.25 * lane_width)

        # leaf nodes
        just_dirive = InArrivalToLocation(
            self.ego_vehicles[0], self._destination_wp.transform.location)
        end_condition = InTriggerDistancePassVehicle(
            self.ego_vehicles[0], self.other_actors[0], 5,
            comparison_operator=operator.gt)
        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(just_dirive)
        root.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StationaryObjectBlockingLane(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectBlockingLane, self).__init__(
            "StationaryObjectBlockingLane",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _start_distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.3}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        # location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location,
                                         carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform)
        static.set_simulate_physics(False)
        self.other_actors.append(static)

        location1, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        location1 -= offset_location
        location1.z += offset['z']
        self.transform1 = carla.Transform(
            location1, carla.Rotation(yaw=orientation_yaw))

        static1 = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform1)
        static1.set_simulate_physics(False)
        self.other_actors.append(static1)

        location2, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        location2 += offset_location
        location2.z += offset['z']
        self.transform2 = carla.Transform(location2,
                                          carla.Rotation(yaw=orientation_yaw))
        static2 = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform2)
        static2.set_simulate_physics(False)
        self.other_actors.append(static2)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[1], self.transform1))
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[2], self.transform2))
        scenario_sequence.add_child(
            TriggerVelocity(self.ego_vehicles[0], 0.001,
                            comparison_operator=operator.lt,
                            name="ego vehicle stop"))
        # scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(StandStill(self.ego_vehicles[0],
                                               name="FinalSpeed",
                                               duration=1))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[1]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[2]))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowWalker(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading
    vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 1.8
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowWalker, self).__init__(
            "FollowWalker",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = \
            get_waypoint_in_distance(self._reference_waypoint,
                                     self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            'walker.*', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" walker. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make other actor to drive until reaching
        the next intersection. Finally, user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel(
            "Waiting for end position",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(
            WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        endcondition.add_child(
            StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(
            ActorTransformSetter(self.other_actors[0],
                                 self._other_actor_transform))
        sequence.add_child(KeepVelocity(self.other_actors[0],
                                        self._first_vehicle_speed,
                                        duration=5))
        sequence.add_child(
            LaneChange(self.other_actors[0],
                       speed=self._first_vehicle_speed,
                       direction='left',
                       distance_other_lane=20))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowCyclist(BasicScenario):

    """
    This class holds everything required for a simple "Follow a
    leading vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 2.8
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowCyclist, self).__init__("FollowCyclist",
                                            ego_vehicles,
                                            config,
                                            world,
                                            debug_mode,
                                            criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(
            self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.diamondback.century', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" walker. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make other actor to drive until reaching
        the next intersection. Finally, user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel(
            "Waiting for end position",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0],
                                                self.other_actors[0]))
        endcondition.add_child(StandStill(self.ego_vehicles[0],
                                          name="FinalSpeed",
                                          duration=1))
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0],
                                                self._other_actor_transform))
        sequence.add_child(KeepVelocity(self.other_actors[0],
                                        self._first_vehicle_speed,
                                        duration=5))
        # sequence.add_child(TimeOut(10))
        sequence.add_child(LaneChange(self.other_actors[0],
                                      speed=self._first_vehicle_speed,
                                      direction='left',
                                      distance_other_lane=20))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CutInUser(BasicScenario):

    """
    This class holds everything required for a simple "Follow a
    leading vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 3.5
        self._delta_velocity = 10
        self._trigger_distance = 7

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CutInUser, self).__init__("CutInUser",
                                        ego_vehicles,
                                        config,
                                        world,
                                        debug_mode,
                                        criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):
        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model,
                                                          actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # transform visible
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 102),
            other_actor_transform.rotation)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # car_visible
        behaviour = py_trees.composites.Sequence(
            "CarOn_{}_Lane".format(self._direction))

        car_visible = ActorTransformSetter(self.other_actors[0],
                                           self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_time = InTimeToArrivalToVehicleWithLaneDiff(
            self.ego_vehicles[0], self.other_actors[0], 5,
            condition_freespace=True)
        just_drive.add_child(trigger_time)
        behaviour.add_child(just_drive)

        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3,
                direction='right', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3, direction='left',
                distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)

        just_drive_a_while = py_trees.composites.Parallel(
                "DrivingAWhile",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        just_drive_a_while.add_child(
            WaypointFollower(self.other_actors[0], self._velocity))
        just_drive_a_while.add_child(TimeOut(3))
        behaviour.add_child(just_drive_a_while)

        # build tree
        root = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        # root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CutOutUser(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading
    vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 4
        self._delta_velocity = 10
        self._trigger_distance = 7

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CutOutUser, self).__init__("CutOutUser",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):
        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model,
                                                          actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # transform visible
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 102),
            other_actor_transform.rotation)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # car_visible
        behaviour = py_trees.composites.Sequence(
            "CarOn_{}_Lane".format(self._direction))

        car_visible = ActorTransformSetter(
            self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_time = TimeOut(5)
        just_drive.add_child(trigger_time)
        behaviour.add_child(just_drive)

        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3,
                direction='right', distance_same_lane=5,
                distance_other_lane=2)
            behaviour.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3,
                direction='left', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)

        just_drive_a_while = py_trees.composites.Parallel(
            "DrivingAWhile",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        just_drive_a_while.add_child(WaypointFollower(self.other_actors[0],
                                                      self._velocity))
        just_drive_a_while.add_child(TimeOut(3))
        behaviour.add_child(just_drive_a_while)

        # build tree
        root = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class BorrowLane(BasicScenario):

    """
    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)

        self._velocity = 30/3.6
        self._delta_velocity = 10
        self._trigger_distance = 7

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(BorrowLane, self).__init__("BorrowLane",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):
        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        _start_distance = 110
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 180, "position": 270, "z": 3, "k": 0.6}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))

        print("location {}".format(location))
        location += offset_location
        location.z += offset['z']
        print("location new{}".format(location))
        self._transform_visible = \
            carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        oppsite_vel = CarlaDataProvider.request_new_actor(
            'vehicle.tesla.model3', self._transform_visible)
        oppsite_vel.set_simulate_physics(True)
        self.other_actors.append(oppsite_vel)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # car_visible
        behaviour = py_trees.composites.Sequence(
            "CarOn_{}_Lane".format(self._direction))

        car_visible = ActorTransformSetter(
            self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = KeepVelocity(self.other_actors[0], self._velocity)
        ego_vehicle_decelerate = InTriggerVelocityDecelerate(
            self.ego_vehicles[0], 5/3.6)
        drive_follow_waypoint = \
            WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)
        just_drive.add_child(ego_vehicle_decelerate)

        behaviour.add_child(just_drive)
        behaviour.add_child(drive_follow_waypoint)

        endcondition = InTriggerDistancePassVehicle(self.ego_vehicles[0],
                                                    self.other_actors[0],
                                                    10,
                                                    freespace=True)

        # build tree
        root = py_trees.composites.Parallel(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        moveon_criterion = MoveOnRunWhenMeetObsTest(self.ego_vehicles[0],
                                                    self.other_actors[0])

        criteria.append(collision_criterion)
        criteria.append(moveon_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StopAndGo(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading
    vehicle" scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 8.3

        # get direction from config name
        self._config = config
        self._transform_visible = None

        super(StopAndGo, self).__init__("StopAndGo",
                                        ego_vehicles,
                                        config,
                                        world,
                                        debug_mode,
                                        criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):
        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model,
                                                          actor.transform)
            if vehicle is None:
                raise RuntimeError("Error: vehicle is None ")
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # transform visible
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 102),
            other_actor_transform.rotation)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # car_visible
        behaviour = py_trees.composites.Sequence("SubBehavior")
        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        car_visible = \
            ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # just_drive with a certain velocity fow a while
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)
        trigger_time = TimeOut(5)
        just_drive.add_child(trigger_time)

        behaviour.add_child(just_drive)
        behaviour.add_child(StopVehicle(self.other_actors[0], 1.0))

        go_condition = py_trees.composites.Parallel(
            "OtherActorGo",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        go_condition.add_child(
            StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
        go_condition.add_child(
            WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))

        behaviour.add_child(go_condition)

        end_condition = py_trees.composites.Parallel(
            "EndBehaviorCondition",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(WaypointFollower(self.other_actors[0], 4.1))
        end_condition.add_child(TimeOut(3))

        behaviour.add_child(end_condition)

        # build tree
        root = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        stand_still_criterion = StandStillTimeTest(self.ego_vehicles[0],
                                                   duration=3)

        criteria.append(collision_criterion)
        criteria.append(stand_still_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StationaryObjectBlockingAllLane(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectBlockingAllLane, self).__init__(
            "StationaryObjectBlockingAllLane",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _start_distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.5}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = \
            waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        # location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location,
                                         carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform)
        static.set_simulate_physics(False)
        self.other_actors.append(static)

        location1, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        location1 += offset_location
        location1.z += offset['z']
        self.transform1 = carla.Transform(location1,
                                          carla.Rotation(yaw=orientation_yaw))

        static1 = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform1)
        static1.set_simulate_physics(False)
        self.other_actors.append(static1)

        location2, _ = get_location_in_distance_from_wp(
            self._reference_waypoint, _start_distance)
        location2 += 2*offset_location
        location2.z += offset['z']
        self.transform2 = carla.Transform(location2,
                                          carla.Rotation(yaw=orientation_yaw))
        static2 = CarlaDataProvider.request_new_actor(
            'static.prop.trafficcone01', self.transform2)
        static2.set_simulate_physics(False)
        self.other_actors.append(static2)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0],
                                                         self.transform))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1],
                                                         self.transform1))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[2],
                                                         self.transform2))
        scenario_sequence.add_child(
            TriggerVelocity(self.ego_vehicles[0], 0.001,
                            comparison_operator=operator.lt,
                            name="ego vehicle stop"))
        scenario_sequence.add_child(StandStill(self.ego_vehicles[0],
                                               name="FinalSpeed",
                                               duration=1))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[1]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[2]))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StopAtFixedLocation(BasicScenario):

    """
    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._map.get_waypoint(config.trigger_points[0].location)
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(StopAtFixedLocation, self).__init__(
            "StopAtFixedLocation",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _calculate_actor_center_to_lane_line(self):
        """
            Please evaluate if need to consider the z axis
        """
        location_diff = self._config.trigger_points[0].location - \
            self._reference_waypoint.transform.location
        rv = self.ego_vehicles[0].get_transform().rotation.get_right_vector()
        distace_to_lane_center = \
            location_diff.x * rv.x + \
            location_diff.y * rv.y + \
            location_diff.z * location_diff.z
        return self._reference_waypoint.lane_width - distace_to_lane_center

    def _initialize_actors(self, config):
        # direction of lane, on which other_actor is driving before lane change
        pass

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        root = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(
            StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1.0))
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        arrive_criterion = InArriveRegionTest(self.ego_vehicles[0],
                                              self._config.destination.x,
                                              self._config.destination.y, 10)

        criteria.append(arrive_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class SpeedLimit(BasicScenario):

    """
    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=20):
        self.timeout = timeout
        self._wmap = CarlaDataProvider.get_map()
        self.max_speed_limit = 30/3.6
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        ts_lm = self._get_landmark(self._reference_waypoint, "274")
        if ts_lm is not None:
            self.traffic_sign_location = ts_lm.transform.location
            self.ref_wp = ts_lm.waypoint
            ts = world.get_traffic_sign(ts_lm)
            logging.info("trigger_volume={}, {}".format(ts, ts.trigger_volume))
        else:
            raise ValueError("Not found traffic sign")

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(SpeedLimit, self).__init__("SpeedLimit",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _get_landmark(self, wp, type, distance=100):
        landmarks = wp.get_landmarks(distance, True)
        for lm in landmarks:
            if type in lm.type:
                logging.info("type={},id={},name={}".format(lm.type,
                                                            lm.id,
                                                            lm.name))
                return lm

        return None

    def _create_behavior(self):
        """
        Order of sequence:
        -
        """
        # Behavior
        behaviour = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        trigger_velocity = TriggerVelocity(self.ego_vehicles[0],
                                           1.75*self.max_speed_limit)
        behaviour.add_child(trigger_velocity)
        behaviour.add_child(
            InArrivalToLocation(self.ego_vehicles[0],
                                self.ref_wp.transform.location))

        # end condition
        self.ref_wp = CarlaDataProvider.get_map().get_waypoint(
            self.traffic_sign_location)
        end_condition = py_trees.composites.Sequence(
            "EndConditon",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(
            InArrivalToLocation(self.ego_vehicles[0],
                                self.ref_wp.transform.location))
        end_condition.add_child(Fail())

        # build tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(end_condition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        speed_criterion = MaxSpeedLimitTest(self.ego_vehicles[0],
                                            self.max_speed_limit)
        location_criterion = ArriveAtLocationTest(
            self.ego_vehicles[0], self.ref_wp.transform.location)

        criteria.append(speed_criterion)
        criteria.append(location_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class LaneKeep(BasicScenario):

    """
    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.speed_require = 30/3.6
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None
        super(LaneKeep, self).__init__("LaneKeep",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _create_behavior(self):
        """
        Order of sequence:
        """
        # Behavior
        behaviour = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        trigger_velocity = TriggerVelocity(
            self.ego_vehicles[0], self.speed_require)

        behaviour.add_child(trigger_velocity)
        behaviour.add_child(TimeOut(self.timeout))

        # build tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        vehicle_run_criterion = VehicleRunTest(self.ego_vehicles[0])
        lane_criterion = KeepLaneTest(self.ego_vehicles[0])
        criteria.append(lane_criterion)
        criteria.append(vehicle_run_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class StopSign(BasicScenario):

    """
    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=20):
        self.timeout = timeout
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = \
            self._wmap.get_waypoint(config.trigger_points[0].location)
        self.speed_require = 30/3.6

        ts_lm = self._get_landmark(self._reference_waypoint, "206")
        if ts_lm is not None:
            self.stop_sign_location = ts_lm.transform.location
            self.ref_wp = ts_lm.waypoint
        else:
            raise ValueError("not found stop sign")
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(StopSign, self).__init__("StopSign",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _get_landmark(self, wp, type, distance=100):
        landmarks = wp.get_landmarks(distance, True)
        for lm in landmarks:
            if type in lm.type:
                logging.info("type={},id={},name={}".format(lm.type,
                                                            lm.id,
                                                            lm.name))
                return lm
        return None

    def _create_behavior(self):
        """
        Order of sequence:
        -
        """
        # Behavior
        behaviour = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        trigger_velocity = TriggerVelocity(self.ego_vehicles[0],
                                           self.speed_require)
        behaviour.add_child(trigger_velocity)

        end_condition = py_trees.composites.Sequence(
            "EndConditon",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(
            InArrivalToLocation(self.ego_vehicles[0],
                                self.ref_wp.transform.location))
        end_condition.add_child(Fail())
        # build tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(end_condition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        vehicle_run_criterion = VehicleRunTest(self.ego_vehicles[0])
        stand_still_criterion = StandStillTimeTest(self.ego_vehicles[0],
                                                   duration=3)
        distance_criterion = ArriveAtLocationTest(
            self.ego_vehicles[0], self.ref_wp.transform.location, 2, False)

        criteria.append(vehicle_run_criterion)
        criteria.append(distance_criterion)
        criteria.append(stand_still_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class WalkerThroughCrosswalk(BasicScenario):

    """
    The ego vehicle is passing through a road,
    And encounters a pedestrian crossing the road. it will stop unit the
    cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=30):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(
            config.trigger_points[0].location)
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8
        self._time_to_reach = 4.5
        self._walker_yaw = 0
        self.wp_walker = self._wmap.get_waypoint(
            carla.Location(x=152.5, y=50),
            lane_type=(carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
        self._location_of_collision = carla.Location(152.6, 62.6)
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        self._ego_vehicle_speed_limit = 10

        super(WalkerThroughCrosswalk, self).__init__(
            "WalkerThroughCrosswalk",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _spawn_adversary(self, transform):

        self._other_actor_target_velocity = 6.5/3.6
        walker = CarlaDataProvider.request_new_actor('walker.*', transform)
        adversary = walker

        return adversary

    def _obtain_left_most_lane_wp(self, waypoint):
        ref_waypoint = waypoint
        ref_dir = ref_waypoint.transform.get_forward_vector()
        is_get_left_lane = True
        for _ in range(100):
            if is_get_left_lane:
                wp_next = ref_waypoint.get_left_lane()
            else:
                wp_next = ref_waypoint.get_right_lane()

            if wp_next is not None:
                if wp_next.lane_type == carla.LaneType.Sidewalk or \
                   wp_next.lane_type == carla.LaneType.Shoulder:
                    return wp_next
                logging.debug("type{}, location{}".format(
                    wp_next.lane_type, wp_next.transform.location))
                wp_dir = wp_next.transform.get_forward_vector()

                # different director
                if wp_dir.x * ref_dir.x + wp_dir.y * ref_dir.y < 0:
                    is_get_left_lane = not is_get_left_lane

                ref_waypoint = wp_next
                ref_dir = ref_waypoint.transform.get_forward_vector()
            else:
                break

        return None

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint = self.wp_walker
        walker = self._spawn_adversary(waypoint.transform)

        self.transform = carla.Transform(
            carla.Location(waypoint.transform.location.x,
                           waypoint.transform.location.y,
                           waypoint.transform.location.z + 1),
            carla.Rotation(yaw=waypoint.transform.rotation.yaw+270))

        # Now that we found a possible position we just put the vehicle to
        # the underground
        disp_transform = carla.Transform(
            carla.Location(waypoint.transform.location.x,
                           waypoint.transform.location.y,
                           waypoint.transform.location.z - 500),
            waypoint.transform.rotation)  # waypoint.transform.rotation.yaw

        walker.set_transform(disp_transform)
        walker.set_simulate_physics(enabled=False)
        self.other_actors.append(walker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="WalkerThroughCrossWalker")

        start_condition = InTimeToArrivalToLocation(
            self.ego_vehicles[0], 4.5+2, self._location_of_collision)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      distance=5*3.5,
                                      name="walker velocity")
        end_condition = DriveDistance(self.ego_vehicles[0], 5)
        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0],
                                 self.transform,
                                 name='TransformSetterTS3walker'))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(actor_velocity)
        scenario_sequence.add_child(end_condition)

        # building tree
        root.add_child(scenario_sequence)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        vehicle_run_criterion = VehicleRunTest(self.ego_vehicles[0])
        stand_still_criterion = StandStillTimeTest(self.ego_vehicles[0],
                                                   self.other_actors[0],
                                                   duration=3)

        criteria.append(vehicle_run_criterion)
        criteria.append(stand_still_criterion)
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CrossTrafficRedLight(BasicScenario):

    """
    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=20):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.speed_require = 30/3.6  # 20km/h
        self.stop_line_location = carla.Location(239.39, 77.61)
        self.ref_wp = CarlaDataProvider.get_map().get_waypoint(
            self.stop_line_location, project_to_road=False)
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CrossTrafficRedLight, self).__init__(
            "CrossTrafficRedLight",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _retrieve_traffic_landmark(self, map: carla.Map,
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

    def _init_traffic_light_state(self, light_state, light_keep_time=60):
        traffic_light = None
        traffic_light_lm = self._retrieve_traffic_landmark(
            self._map, self.ego_vehicles[0], 100)

        for tl_lm in traffic_light_lm:
            traffic_light = \
                CarlaDataProvider.get_world().get_traffic_light(tl_lm)
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

    def _create_behavior(self):
        """
        Order of sequence:
        """

        set_traffic_light_state(self._map, self.ego_vehicles[0],
                                carla.TrafficLightState.Green)

        # Behavior
        behaviour = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # behaviour.add_child(trigger_velocity)
        behaviour.add_child(InTriggerDistanceToLocationSetTrafficLight(
            self.ego_vehicles[0], self.ref_wp.transform.location, 40))
        behaviour.add_child(StandStill(self.ego_vehicles[0],
                                       name="CheckStankStill",
                                       duration=3))
        behaviour.add_child(DriveDistance(self.ego_vehicles[0], 5))

        end_condition = py_trees.composites.Sequence(
            "EndConditon",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(InArrivalToLocation(
            self.ego_vehicles[0], self.ref_wp.transform.location))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], 5))
        end_condition.add_child(Fail())
        # build tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(end_condition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        vehicle_run_criterion = VehicleRunTest(self.ego_vehicles[0])
        stand_still_criterion = StandStillTimeTest(self.ego_vehicles[0],
                                                   duration=3)
        distance_criterion = StandStillDistanceTest(
            self.ego_vehicles[0], self.ref_wp.transform.location, 2)

        criteria.append(vehicle_run_criterion)
        criteria.append(distance_criterion)
        criteria.append(stand_still_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CrossTrafficGreenLight(BasicScenario):

    """
    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=30):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.speed_require = 30/3.6  # 30km/h
        self.stop_line_location = carla.Location(239.39, 77.61)
        self.ref_wp = CarlaDataProvider.get_map().get_waypoint(
            self.stop_line_location, project_to_road=False)
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CrossTrafficGreenLight, self).__init__(
            "CrossTrafficGreenLight",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _create_behavior(self):
        """
        Order of sequence:
        -
        """

        set_traffic_light_state(self._map, self.ego_vehicles[0],
                                carla.TrafficLightState.Green)

        # Behavior
        behaviour = py_trees.composites.Sequence(
            "Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        trigger_velocity = TriggerVelocity(self.ego_vehicles[0],
                                           self.speed_require)
        behaviour.add_child(trigger_velocity)
        behaviour.add_child(InArrivalToLocation(
            self.ego_vehicles[0], self.ref_wp.transform.location))

        end_condition = py_trees.composites.Sequence(
            "EndConditon",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(InArrivalToLocation(
            self.ego_vehicles[0], self.ref_wp.transform.location))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], 5))
        end_condition.add_child(Fail())

        # build tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(end_condition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the
        parallel behavior tree.
        """
        criteria = []

        vehicle_run_criterion = VehicleRunTest(self.ego_vehicles[0])
        vehicle_never_stop_criterion = NeverStopTest(self.ego_vehicles[0])
        criteria.append(vehicle_run_criterion)
        criteria.append(vehicle_never_stop_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class NoSignalJunctionCrossingGoStraight(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout
        self.destination = config.destination
        self.ref_wp = \
            CarlaDataProvider.get_map().get_waypoint(self.destination)

        super(NoSignalJunctionCrossingGoStraight, self).__init__(
            "NoSignalJunctionCrossingGoStraight",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -80, -70,
            -100, -95)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-139.84))  # collision may happen here

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],
            -110, -100,
            -142, -135)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InArrivalToLocation(self.ego_vehicles[0],
                                            self.ref_wp.transform.location)

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0],
                                 self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        location_criterion = ArriveAtLocationTest(
            self.ego_vehicles[0], self.ref_wp.transform.location,
            min_distance=1)

        criteria.append(collison_criteria)
        criteria.append(location_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class NoSignalJunctionCrossingTurnRight(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout
        self.destination = config.destination
        self.ref_wp = \
            CarlaDataProvider.get_map().get_waypoint(self.destination)

        super(NoSignalJunctionCrossingTurnRight, self).__init__(
            "NoSignalJunctionCrossingTurnRight",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -80, -70,
            -100, -95)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-136.34))  # collision may happen here

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],
            -10, -5,
            -142, -134)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InArrivalToLocation(
            self.ego_vehicles[0], self.ref_wp.transform.location)

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(
            ActorTransformSetter(self.other_actors[0],
                                 self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
        scenario_sequence.add_child(end_condition)

        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        location_criterion = ArriveAtLocationTest(
            self.ego_vehicles[0], self.ref_wp.transform.location,
            min_distance=1)
        light_statue_criterion = VehicleTurnSignalTest(
            self.ego_vehicles[0], carla.VehicleLightState.RightBlinker)
        criteria.append(collison_criteria)
        criteria.append(location_criterion)
        criteria.append(light_statue_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class NoSignalJunctionCrossingTurnLeft(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout
        self.destination = config.destination
        self.ref_wp = \
            CarlaDataProvider.get_map().get_waypoint(self.destination)

        super(NoSignalJunctionCrossingTurnLeft, self).__init__(
            "NoSignalJunctionCrossingTurnLeft",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(
            config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -80, -70,
            -100, -95)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-88.74, y=-140.23))  # collision may happen here

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],
            -90, -80,
            -115, -105)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InArrivalToLocation(
            self.ego_vehicles[0], self.ref_wp.transform.location)

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(
            self.other_actors[0], self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        location_criterion = ArriveAtLocationTest(
            self.ego_vehicles[0], self.ref_wp.transform.location,
            min_distance=1)
        light_statue_criterion = VehicleTurnSignalTest(
            self.ego_vehicles[0], carla.VehicleLightState.LeftBlinker)

        criteria.append(collison_criteria)
        criteria.append(location_criterion)
        criteria.append(light_statue_criterion)
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
