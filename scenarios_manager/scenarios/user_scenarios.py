#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

import math
import random
import py_trees
from py_trees.common import ParallelPolicy
import carla
import operator                                                                           

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      LaneChange)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance, InTriggerDistanceToOSCPosition,
                                                                               StandStill,
                                                                               TriggerVelocity,
                                                                               InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               WaitUntilInFront)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (get_location_in_distance_from_wp, get_waypoint_in_distance)

from scenarios_manager.scenarioatomics.atomic_trigger_conditions import (TriggerInSameLane,
                                                                        InTimeToArrivalToLocationBasedOnMaxSpeed,
                                                                        InTimeToArrivalToVehicleWithLaneDiff,
                                                                        InTriggertoVehiclePassDistance)
from scenarios_manager.scenarioatomics.atomic_behaviors import DecelerateToVelocity
from scenarios_manager.scenarioatomics.atomic_criteria import  InArriveRegionTest

class StationaryVehicleCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryVehicleCrossing, self).__init__("StationaryVehicleCrossing",
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
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', self.transform)
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
        actor_stand = TimeOut(15)
        actor_removed = ActorDestroy(self.other_actors[0])
        end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven)

        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(actor_removed)
        scenario_sequence.add_child(end_condition)

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
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        # config.trigger_points[0].location is about 'ego vehicle loaction in config file'
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40 # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type  # flag to select either pedestrian (False) or cyclist (True)
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
        self._ego_vehicle_speed_limit = 10 # assume 36km/h (about 16.7 = 60km/h)
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(DynamicObjectCrossingWithStop, self).__init__("DynamicObjectCrossingWithStop",
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

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        # self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            # self._other_actor_target_velocity = 3 + (0.4 * self._num_lane_changes)
            self._other_actor_target_velocity = 1.8 # 6.5km/h
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            # self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
            self._other_actor_target_velocity = 4.5 # 16km/h
            first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
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

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self.transform2 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit # assume ego vehicle max speed = 60km/h
        
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

        self._other_actor_stop_location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance, True)
        if self._other_actor_stop_location is not None:
            print("Predict collision place[x({}),y({})], start_distance{}".format(self._other_actor_stop_location.x,
                                                                                  self._other_actor_stop_location.y,
                                                                                  _start_distance))
        else:
            raise Exception

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform, orientation_yaw)

                blocker = self._spawn_blocker(self.transform, orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
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
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self.transform.location,
                                                                    dist_to_trigger)
        else:
            # start_condition = InTimeToArrivalToVehicle(self.ego_vehicles[0],
            #                                            self.other_actors[0],
            #                                            self._time_to_reach)  # the ego vehicle and other actor will occur collision in _time_to_reach
            start_condition = InTimeToArrivalToLocationBasedOnMaxSpeed(self.ego_vehicles[0],
                                                                       self._time_to_reach + 2.0,
                                                                       self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        # actor_drive_to_ego_lane_id = TriggerInSameLane(self.ego_vehicles[0], self.other_actors[0])
        # other_actor_drive_distance = math.fabs(self.transform.location.y - self._reference_waypoint.transform.location.y)
        other_actor_drive_distance = self.transform.location.distance(self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
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
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0], name="walker stand still")
        actor_continue_condition = TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")
        # continue_drive = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="continue drive")        
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep stand")
        
        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform,
                                                         name='TransformSetterTS3walker'))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self.transform2,
                                                         name='TransformSetterTS3coca', physics=False))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))
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
        # continue_drive.add_child(actor_velocity)
        # continue_drive.add_child(actor_drive)
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
    And encounters a cyclist/pedestrian crossing the road. it will stop unit the cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        # config.trigger_points[0].location is about 'ego vehicle loaction in config file'
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40 # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type  # flag to select either pedestrian (False) or cyclist (True)
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
        self._ego_vehicle_speed_limit = 10 # assume 36km/h (about 16.7 = 60km/h)
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(WalkerCrossingWithStop, self).__init__("WalkerCrossingWithStop",
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

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        # self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            # self._other_actor_target_velocity = 3 + (0.4 * self._num_lane_changes)
            self._other_actor_target_velocity = 1.8 # 6.5km/h
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            # self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
            self._other_actor_target_velocity = 4.5 # 16km/h
            first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
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

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self.transform2 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit # assume ego vehicle max speed = 60km/h
        
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

        self._other_actor_stop_location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance, True)
        if self._other_actor_stop_location is not None:
            print("Predict collision place[x({}),y({})], start_distance{}".format(self._other_actor_stop_location.x,
                                                                                  self._other_actor_stop_location.y,
                                                                                  _start_distance))
        else:
            raise Exception

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform, orientation_yaw)

                blocker = self._spawn_blocker(self.transform, orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
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
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self.transform.location,
                                                                    dist_to_trigger)
        else:
            # start_condition = InTimeToArrivalToVehicle(self.ego_vehicles[0],
            #                                            self.other_actors[0],
            #                                            self._time_to_reach)  # the ego vehicle and other actor will occur collision in _time_to_reach
            start_condition = InTimeToArrivalToLocationBasedOnMaxSpeed(self.ego_vehicles[0],
                                                                       self._time_to_reach + 2.0,
                                                                       self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        # actor_drive_to_ego_lane_id = TriggerInSameLane(self.ego_vehicles[0], self.other_actors[0])
        # other_actor_drive_distance = math.fabs(self.transform.location.y - self._reference_waypoint.transform.location.y)
        other_actor_drive_distance = self.transform.location.distance(self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        actor_remove = ActorDestroy(self.other_actors[0],
                                    name="Destroying walker")
        # static_remove = ActorDestroy(self.other_actors[1],
        #                              name="Destroying Prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0], name="walker stand still")
        actor_continue_condition = TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")
        # continue_drive = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="continue drive")        
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep stand")
        
        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform,
                                                         name='TransformSetterTS3walker'))
        # scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self.transform2,
        #                                                  name='TransformSetterTS3coca', physics=False))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))
        scenario_sequence.add_child(keep_velocity)
        # scenario_sequence.add_child(continue_drive)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(keep_stand)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(actor_remove)
        # scenario_sequence.add_child(static_remove)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        # continue_drive.add_child(actor_velocity)
        # continue_drive.add_child(actor_drive)
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

class WalkerCrossingWithPropAndStop(BasicScenario):

    """    
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road. it will stop unit the cyclist/pedestrian walks away.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        # config.trigger_points[0].location is about 'ego vehicle loaction in config file'
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40 # total drive distance
        # other vehicle parameters
        self._other_actor_target_velocity = 1.8  # 1.8m/s≈6.5km/h
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 4.5
        self._adversary_type = adversary_type  # flag to select either pedestrian (False) or cyclist (True)
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
        self._ego_vehicle_speed_limit = 10 # assume 36km/h (about 16.7 = 60km/h)
        self._other_actor_stop_location = carla.Location()
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(WalkerCrossingWithPropAndStop, self).__init__("WalkerCrossingWithPropAndStop",
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

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        # location += offset_location # spawn vehicle in front of the prop(vehicle)
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _calculate_prop_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        orientation_yaw = waypoint.transform.rotation.yaw        
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):

        # self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            # self._other_actor_target_velocity = 3 + (0.4 * self._num_lane_changes)
            self._other_actor_target_velocity = 1.8 # 6.5km/h
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            # self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
            self._other_actor_target_velocity = 4.5 # 16km/h
            first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
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

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self.transform2 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw))

        static = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = self._time_to_reach * self._ego_vehicle_speed_limit # assume ego vehicle max speed = 60km/h
        
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

        self._other_actor_stop_location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance, True)
        if self._other_actor_stop_location is not None:
            print("Predict collision place[x({}),y({})], start_distance{}".format(self._other_actor_stop_location.x,
                                                                                  self._other_actor_stop_location.y,
                                                                                  _start_distance))
        else:
            raise Exception

        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform, orientation_yaw)

                blocker_base_transform, blocker_orientation_yaw = self._calculate_prop_base_transform(_start_distance, waypoint)
                blocker = self._spawn_blocker(blocker_base_transform, blocker_orientation_yaw)

                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
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
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self.transform.location,
                                                                    dist_to_trigger)
        else:
            # start_condition = InTimeToArrivalToVehicle(self.ego_vehicles[0],
            #                                            self.other_actors[0],
            #                                            self._time_to_reach)  # the ego vehicle and other actor will occur collision in _time_to_reach
            start_condition = InTimeToArrivalToLocationBasedOnMaxSpeed(self.ego_vehicles[0],
                                                                       self._time_to_reach + 2.0,
                                                                       self._other_actor_stop_location)

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        # actor_drive_to_ego_lane_id = TriggerInSameLane(self.ego_vehicles[0], self.other_actors[0])
        # other_actor_drive_distance = math.fabs(self.transform.location.y - self._reference_waypoint.transform.location.y)
        other_actor_drive_distance = self.transform.location.distance(self._other_actor_stop_location)
        actor_drive = DriveDistance(self.other_actors[0],
                                    other_actor_drive_distance,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
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
                                      name="End condition ego drive distance")
        actor_stand_still = StandStill(self.other_actors[0], name="walker stand still")
        actor_continue_condition = TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="walker continue condition")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")
        # continue_drive = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="continue drive")        
        keep_stand = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep stand")
        
        # building tree

        root.add_child(scenario_sequence)
        # scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform,
        #                                                  name='TransformSetterTS3walker'))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self.transform2,
                                                         name='TransformSetterTS3coca', physics=False))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform,
                                                         name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))        
        scenario_sequence.add_child(keep_velocity) 
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

class FollowLeadingVehicleWithLanechangeAndObstacle(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle, 
    so that the leading vehicle will change lane then.

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 25
        self._second_actor_location = self._first_actor_location + 60
        self._first_actor_speed = 11  # about 40km/h
        self._second_actor_speed = 0
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None
        self._direction = 'left'

        super(FollowLeadingVehicleWithLanechangeAndObstacle, self).__init__("FollowLeadingVehicleWithLanechangeAndObstacle",
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

        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
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
        # yaw_1 = second_actor_waypoint.transform.rotation.yaw + 90
        yaw_1 = second_actor_waypoint.transform.rotation.yaw
        second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z - 500),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
                           second_actor_waypoint.transform.rotation.roll))
        self._second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z + 1),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
                           second_actor_waypoint.transform.rotation.roll))

        first_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_actor_transform)
        # second_actor = CarlaDataProvider.request_new_actor(
        #     'vehicle.diamondback.century', second_actor_transform)
        second_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', second_actor_transform)

        first_actor.set_simulate_physics(enabled=False)
        second_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(first_actor)
        self.other_actors.append(second_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacle.
        Once obstacle clears the road, make the other actor to drive towards the
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
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self.other_actors[1],
                                                                          self.other_actors[0], distance=30))
        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], direction='right')            
        else:
            lane_change = LaneChange(
                self.other_actors[0], direction='left')

        # end condition
        # endcondition = py_trees.composites.Parallel("Waiting for end position",
        #                                             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        # endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
        #                                                 self.ego_vehicles[0],
        #                                                 distance=20,
        #                                                 name="FinalDistance")
        # endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        # endcondition.add_child(endcondition_part1)
        # endcondition.add_child(endcondition_part2)
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[1]))
        endcondition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))                                      
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        sequence.add_child(AccelerateToVelocity(self.other_actors[0],
                                                1.0,
                                                self._first_actor_speed))                                        
        sequence.add_child(driving_to_next_intersection)
        # sequence.add_child(TimeOut(3))
        # sequence.add_child(obstacle_clear_road)
        sequence.add_child(LaneChange(self.other_actors[0]))
        # sequence.add_child(DriveDistance(self.other_actors[0], 10))
        # sequence.add_child(stop_near_intersection)
        # sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
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
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 14 # 50km/h
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 0.8
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleWithDecelerate, self).__init__("FollowLeadingVehicleWithDecelerate",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
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
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        endcondition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))                                      
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(AccelerateToVelocity(self.other_actors[0],
                                                1.0,
                                                self._first_vehicle_speed))                                        
        sequence.add_child(DriveDistance(self.other_actors[0], 30))
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        # sequence.add_child(TimeOut(3))
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

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryVehicleCrossingLane, self).__init__("StationaryVehicleCrossingLane",
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
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.8}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', self.transform)
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
        actor_stand_still = StandStill(self.other_actors[0], name="walker stand still")
        actor_continue_condition = TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="walker continue condition")                                                    
        actor_stand = TimeOut(15)
        actor_removed = ActorDestroy(self.other_actors[0])
        end_condition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        end_condition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
        
        # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="ego vehicle stop"))
        # scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(actor_removed)

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

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectBlockingLane, self).__init__("StationaryObjectBlockingLane",
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
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.3}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        # location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform)
        static.set_simulate_physics(False)
        self.other_actors.append(static)
        
        location1, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        location1 -= offset_location 
        location1.z += offset['z']  
        self.transform1 = carla.Transform(location1, carla.Rotation(yaw=orientation_yaw))

        static1 = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform1)
        static1.set_simulate_physics(False)
        self.other_actors.append(static1)

        location2, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        location2 += offset_location
        location2.z += offset['z']  
        self.transform2 = carla.Transform(location2, carla.Rotation(yaw=orientation_yaw))
        static2 = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform2)
        static2.set_simulate_physics(False)
        self.other_actors.append(static2)

        print("offset_location[x({}),y({}),z({})]".format(offset_location.x, offset_location.y, offset_location.z))
        print("location[x({}),y({}),z({})]".format(location.x, location.y, location.z))
        print("location1[x({}),y({}),z({})]".format(location1.x, location1.y, location1.z))
        print("location2[x({}),y({}),z({})]".format(location2.x, location2.y, location2.z))


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
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self.transform1))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[2], self.transform2))
        scenario_sequence.add_child(TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="ego vehicle stop"))
        # scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
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
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 1.8 # 6.5km/h
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowWalker, self).__init__("FollowWalker",
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

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
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
        first_vehicle = CarlaDataProvider.request_new_actor('walker.*', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" walker. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        endcondition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))                                      
        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(KeepVelocity(self.other_actors[0],
                                        self._first_vehicle_speed,
                                        duration=5))                                        
        # sequence.add_child(TimeOut(10))
        sequence.add_child(LaneChange(self.other_actors[0], 
                            speed=self._first_vehicle_speed, 
                            direction='left',
                            distance_other_lane = 20))
        # sequence.add_child(ActorDestroy(self.other_actors[0]))
        # sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        # sequence.add_child(TimeOut(3))
        # sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
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
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 2.8 # 10km/h
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
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

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
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
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" walker. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        endcondition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        endcondition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))                                      
        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(KeepVelocity(self.other_actors[0],
                                        self._first_vehicle_speed,
                                        duration=5))                                        
        # sequence.add_child(TimeOut(10))
        sequence.add_child(LaneChange(self.other_actors[0], 
                            speed=self._first_vehicle_speed, 
                            direction='left',
                            distance_other_lane = 20))
        # sequence.add_child(TimeOut(3))
        # sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
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
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 3.5 # About 7 = 25km/h
        self._delta_velocity = 10
        self._trigger_distance = 7 # (30-25) * 5 / 3.6 

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
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
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
        behaviour = py_trees.composites.Sequence("CarOn_{}_Lane" .format(self._direction))

        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        trigger_time = InTimeToArrivalToVehicleWithLaneDiff(self.ego_vehicles[0], self.other_actors[0], 5, condition_freespace=True)
        just_drive.add_child(trigger_time)
        behaviour.add_child(just_drive)        

        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3, direction='right', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3, direction='left', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)

        just_drive_a_while = py_trees.composites.Parallel("DrivingAWhile", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        just_drive_a_while.add_child(WaypointFollower(self.other_actors[0], self._velocity))
        just_drive_a_while.add_child(TimeOut(3))
        behaviour.add_child(just_drive_a_while)

        # endcondition
        # endcondition = DriveDistance(self.other_actors[0], 50)
        # build tree
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        # root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
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
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 4 # About 7 = 25km/h
        self._delta_velocity = 10
        self._trigger_distance = 7 # (30-25) * 5 / 3.6 

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
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
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
        behaviour = py_trees.composites.Sequence("CarOn_{}_Lane" .format(self._direction))

        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        # trigger_time = InTimeToArrivalToVehicleWithLaneDiff(self.ego_vehicles[0], self.other_actors[0], 5, condition_freespace=True)
        trigger_time = TimeOut(5)
        just_drive.add_child(trigger_time)
        behaviour.add_child(just_drive)        

        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3, direction='right', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=self._velocity*3, direction='left', distance_same_lane=5, distance_other_lane=2)
            behaviour.add_child(lane_change)

        just_drive_a_while = py_trees.composites.Parallel("DrivingAWhile", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        just_drive_a_while.add_child(WaypointFollower(self.other_actors[0], self._velocity))
        just_drive_a_while.add_child(TimeOut(3))
        behaviour.add_child(just_drive_a_while)

        # endcondition
        endcondition = DriveDistance(self.ego_vehicles[0], 20)
        # build tree
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
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

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 8 # About 7 = 25km/h
        self._delta_velocity = 10
        self._trigger_distance = 7 # (30-25) * 5 / 3.6 

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

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
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
        behaviour = py_trees.composites.Sequence("CarOn_{}_Lane" .format(self._direction))

        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)

        # start condition
        startcondition = TriggerVelocity(self.ego_vehicles[0], 1)
        behaviour.add_child(startcondition)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        car_driving = KeepVelocity(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggertoVehiclePassDistance(self.ego_vehicles[0], self.other_actors[0], condition_freespace=True)
        just_drive.add_child(trigger_distance)
        behaviour.add_child(just_drive)  

        # endcondition
        # endcondition = DriveDistance(self.ego_vehicles[0], 20)
        # build tree
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        # root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
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

class StopAndGo(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 8.3 # About 30km/h

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
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
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

        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behaviour.add_child(car_visible)       

        # just_drive with a certain velocity fow a while
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)
        trigger_time = TimeOut(5)
        just_drive.add_child(trigger_time)
        # trigger_velocity = TriggerVelocity(self.ego_vehicles[0], self._velocity)
        # just_drive.add_child(trigger_velocity)

        behaviour.add_child(just_drive)
        behaviour.add_child(StopVehicle(self.other_actors[0], 1.0))

        go_condition = py_trees.composites.Parallel("OtherActorGo", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        go_condition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
        go_condition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        
        behaviour.add_child(go_condition)

        end_condition = py_trees.composites.Parallel("EndBehaviorCondition", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(WaypointFollower(self.other_actors[0], 4.1))
        end_condition.add_child(TimeOut(3))

        behaviour.add_child(end_condition)
        # endcondition1 = py_trees.composites.Parallel("OtherActorGo", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        # endcondition1.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
        # endcondition1.add_child(AccelerateToVelocity(self.other_actors[0], 1.0, 4.1))
        # endcondition1.add_child(DriveDistance(self.other_actors[0], 20))

        # endcondition2 = py_trees.composites.Sequence("EgoVehiclePass", ParallelPolicy = py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        # endcondition2.add_child(TriggerVelocity(self.ego_vehicles[0], 1))
        # endcondition2.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[0]))
        # endcondition2.add_child(DriveDistance(self.ego_vehicles[0], 20))

        # endcondition
        # endcondition = py_trees.composites.Parallel("EndCondition", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # endcondition.add_child(endcondition1)
        # endcondition.add_child(endcondition2)

        # build tree
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        # root.add_child(TimeOut(3))
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
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

class StationaryObjectBlockingAllLane(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectBlockingAllLane, self).__init__("StationaryObjectBlockingAllLane",
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
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 330, "position": 90, "z": 0.4, "k": 0.5}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        # location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform)
        static.set_simulate_physics(False)
        self.other_actors.append(static)
        
        location1, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        location1 += offset_location 
        location1.z += offset['z']  
        self.transform1 = carla.Transform(location1, carla.Rotation(yaw=orientation_yaw))

        static1 = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform1)
        static1.set_simulate_physics(False)
        self.other_actors.append(static1)

        location2, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
        location2 += 2*offset_location
        location2.z += offset['z']  
        self.transform2 = carla.Transform(location2, carla.Rotation(yaw=orientation_yaw))
        static2 = CarlaDataProvider.request_new_actor('static.prop.trafficcone01', self.transform2)
        static2.set_simulate_physics(False)
        self.other_actors.append(static2)

        # print("offset_location[x({}),y({}),z({})]".format(offset_location.x, offset_location.y, offset_location.z))
        # print("location[x({}),y({}),z({})]".format(location.x, location.y, location.z))
        # print("location1[x({}),y({}),z({})]".format(location1.x, location1.y, location1.z))
        # print("location2[x({}),y({}),z({})]".format(location2.x, location2.y, location2.z))


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
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self.transform))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self.transform1))
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[2], self.transform2))
        scenario_sequence.add_child(TriggerVelocity(self.ego_vehicles[0], 0.001, 
                                                    comparison_operator=operator.lt, name="ego vehicle stop"))
        # scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
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

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        print("reference_point x[{}],y[{}],z[{}]".format(self._reference_waypoint.transform.location.x,
                                                          self._reference_waypoint.transform.location.y,
                                                          self._reference_waypoint.transform.location.z))

        print("original_point x[{}],y[{}],z[{}]".format(config.trigger_points[0].location.x,
                                                          config.trigger_points[0].location.y,
                                                          config.trigger_points[0].location.z))
        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(StopAtFixedLocation, self).__init__("StopAtFixedLocation",
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
        location_diff = self._config.trigger_points[0].location - self._reference_waypoint.transform.location
        rv = self.ego_vehicles[0].get_transform().rotation.get_right_vector()
        distace_to_lane_center = location_diff.x * rv.x + location_diff.y * rv.y + location_diff.z * location_diff.z
        print("distace_to_lane_center = {} to_lane={}".format(distace_to_lane_center, self._reference_waypoint.lane_width - distace_to_lane_center))
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
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1.0))
        # root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        arrive_criterion = InArriveRegionTest(self.ego_vehicles[0], self._config.destination[0], 
                                              self._config.destination[1], 10)
                                            
        criteria.append(arrive_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()