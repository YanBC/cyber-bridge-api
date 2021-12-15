import math
import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion

class InArriveRegionTest(Criterion):

    """
    The test is a success if the actor is within a given radius of a specified region

    Important parameters:
    - actor: CARLA actor to be used for this test
    - x, y, radius: Position (x,y) and radius (in meters) used to get the checked region
    - lateral_distance: the distance between the actor right edge and the position
    """

    def __init__(self, actor, x, y, radius = 10, lateral_distance = 0.3, name="InArriveRegionTest"):
        """
        """
        super(InArriveRegionTest, self).__init__(name, actor, (10, 0.3))
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._x = x     # pylint: disable=invalid-name
        self._y = y     # pylint: disable=invalid-name
        self._radius = radius
        self._lateral_distance = lateral_distance

    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING
        actor_extent_y = 0

        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            return new_status

        rv = self._actor.get_transform().rotation.get_right_vector()
        lateral_diff  = (location.x - self._x) * rv.x + (location.y - self._y) * rv.y
        if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
                actor_extent_y = self._actor.bounding_box.extent.y

        if self.test_status != "SUCCESS":
            xy_distance = math.sqrt(((location.x - self._x)**2) + ((location.y - self._y)**2))
            in_radius = xy_distance < self._radius
            close_to_lane =  math.fabs(lateral_diff) <= actor_extent_y + self._lateral_distance
            not_in_lane = math.fabs(lateral_diff) > actor_extent_y
            self.actual_value = (round(xy_distance, 2), \
                                round(math.fabs(math.fabs(lateral_diff) - actor_extent_y), 2))

            if in_radius and close_to_lane and not_in_lane:
                route_completion_event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETED)
                route_completion_event.set_message("Destination was successfully reached")
                self.list_traffic_events.append(route_completion_event)                
                self.test_status = "SUCCESS"
            else:
                self.test_status = "RUNNING"

        if self.test_status == "SUCCESS":
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        print("extend y[{}], lateral[{}]".format(actor_extent_y, lateral_diff))

        return new_status

    def terminate(self, new_status):
        """
        Set final status
        """
        if self.test_status == "RUNNING":
            self.test_status = "FAILURE"
        super(InArriveRegionTest, self).terminate(new_status)