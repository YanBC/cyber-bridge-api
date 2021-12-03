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
import datetime
import math
import py_trees
import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.tools.scenario_helper import get_distance_along_route
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition

import srunner.tools as sr_tools

EPSILON = 0.001

# class AtomicCondition(py_trees.behaviour.Behaviour):

#     """
#     Base class for all atomic conditions used to setup a scenario

#     *All behaviors should use this class as parent*

#     Important parameters:
#     - name: Name of the atomic condition
#     """

#     def __init__(self, name):
#         """
#         Default init. Has to be called via super from derived class
#         """
#         super(AtomicCondition, self).__init__(name)
#         self.logger.debug("%s.__init__()" % (self.__class__.__name__))
#         self.name = name

#     def setup(self, unused_timeout=15):
#         """
#         Default setup
#         """
#         self.logger.debug("%s.setup()" % (self.__class__.__name__))
#         return True

#     def initialise(self):
#         """
#         Initialise setup
#         """
#         self.logger.debug("%s.initialise()" % (self.__class__.__name__))

#     def terminate(self, new_status):
#         """
#         Default terminate. Can be extended in derived class
#         """
#         self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

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
        comparison_operator: Type "operator", used to compare the two velocities
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
        Gets the speed of the two actors and compares them according to the comparison operator

        returns:
            py_trees.common.Status.RUNNING when the comparison fails and
            py_trees.common.Status.SUCCESS when it succeeds
        """
        new_status = py_trees.common.Status.RUNNING

        actor_waypoint = self._map.get_waypoint(self._actor.get_location(), lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
        other_actor_waypoint = self._map.get_waypoint(self._other_actor.get_location(), lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))

        if actor_waypoint.road_id == other_actor_waypoint.road_id and actor_waypoint.lane_id == other_actor_waypoint.lane_id:
            # print("actor id{}, {}, land_id {}, {}".format(actor_waypoint.road_id, other_actor_waypoint.road_id, actor_waypoint.lane_id , other_actor_waypoint.lane_id ))
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status