from datetime import datetime
import math
import py_trees

import carla

from srunner.scenariomanager.timer import GameTime
from atomic_trigger_conditions import DriveDistance
from scenarios_manager.scenarioatomics.atomic_trigger_conditions import EPSILON

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion

class VehicleRunTest(Criterion):

    """
    This class contains an atomic test for judge if the vehicle runs.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, optional=False, name="CheckVehicleRunTest"):
        """
        Setup actor and maximum allowed velovity
        """
        self.init_location = carla.Location()
        super(VehicleRunTest, self).__init__(name, actor, True, None, optional)

    def update(self):
        """
        Check location
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        location = CarlaDataProvider.get_location(self.actor)
        # self.init_location = location if self.init_location == carla.Location()
        if self.init_location == carla.Location():
            self.init_location = location
       
        distance_vector = self.init_location - location
        distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

        if distance <= 2:
            self.actual_value = False
            self.test_status = "FAILURE"
        else:
            self.actual_value = True
            self.test_status = "SUCCESS"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status
    
    def terminate(self, new_status):
        if (self.test_status == "RUNNING") or (self.test_status == "INIT"):
            self.test_status = "FAILURE"
        return super().terminate(new_status)

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
        self._radius = radius
        self._lateral_distance = lateral_distance 
        
        self._x = self._calc_close_to_sidewalk_ref_point(x, y).x # pylint: disable=invalid-name
        self._y = self._calc_close_to_sidewalk_ref_point(x, y).y # pylint: disable=invalid-name

    def _calc_close_to_sidewalk_ref_point(self, x, y):
        ref_location = carla.Location(x, y)
        waypoint = CarlaDataProvider.get_map().get_waypoint(ref_location)
        position_yaw = waypoint.transform.rotation.yaw
        offset_angel = position_yaw + 90 # right side of vehicle 
        lane_width = waypoint.lane_width
        offset_location = carla.Location(0.5 * lane_width * math.cos(math.radians(offset_angel)),
                                         0.5 * lane_width * math.sin(math.radians(offset_angel)))

        print("debug waypoint x[{}], y[{}]".format(waypoint.transform.location.x, waypoint.transform.location.y))
        print("debug offset x[{}], y[{}]".format(offset_location.x, offset_location.y))
        return waypoint.transform.location + offset_location

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
        lateral_diff  = (location.x - self._x) * rv.x + (location.y - self._y) * rv.y  # vehicle location reference point vs destination point
        if isinstance(self._actor, (carla.Vehicle, carla.Walker)):
                actor_extent_y = self._actor.bounding_box.extent.y

        if self.test_status != "SUCCESS":
            xy_distance = math.sqrt(((location.x - self._x)**2) + ((location.y - self._y)**2))
            in_radius = xy_distance < self._radius
            close_to_lane =  math.fabs(lateral_diff) <= actor_extent_y + self._lateral_distance
            not_in_sidewalk = math.fabs(lateral_diff) > actor_extent_y
            self.actual_value = (round(xy_distance, 2), \
                                round(math.fabs(math.fabs(lateral_diff) - actor_extent_y), 2))
            actor_keep_stop = CarlaDataProvider.get_velocity(self._actor) <= 0.001

            if actor_keep_stop and in_radius and close_to_lane and not_in_sidewalk:
                route_completion_event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETED)
                route_completion_event.set_message("Destination was successfully reached")
                self.list_traffic_events.append(route_completion_event)                
                self.test_status = "SUCCESS"
            else:
                self.test_status = "RUNNING"

        if self.test_status == "SUCCESS":
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        # print("vehicle width[{}*2], lateral diff to destination[{}]".format(actor_extent_y, lateral_diff))

        return new_status

    def terminate(self, new_status):
        """
        Set final status
        """
        if self.test_status == "RUNNING":
            self.test_status = "FAILURE"
        super(InArriveRegionTest, self).terminate(new_status)

class MaxSpeedLimitTest(Criterion):

    """
    This class contains an atomic test for maximum velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - max_velocity_allowed: maximum allowed velocity in m/s
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, max_velocity_allowed, optional=False, name="CheckMaximumVelocity"):
        """
        Setup actor and maximum allowed velovity
        """
        self.greater_than_speed_lower_limit = False
        super(MaxSpeedLimitTest, self).__init__(name, actor, round(max_velocity_allowed,2), None, optional)

    def update(self):
        """
        Check velocity
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        velocity = CarlaDataProvider.get_velocity(self.actor)

        self.actual_value = round(max(velocity, self.actual_value), 2)

        if velocity > self.expected_value_success:
            self.test_status = "FAILURE"
        else:
            self.test_status = "SUCCESS"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status
    
    def terminate(self, new_status):
        if (self.test_status == "RUNNING") or (self.test_status == "INIT"):
            self.test_status = "FAILURE"
        return super().terminate(new_status)

class ArriveAtLocationTest(Criterion):

    """
    This class contains an atomic test for maximum velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - distance: allow the actor keep min distance with the destination
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, destination, min_distance = 0.01, allow_exceed = True,  optional=False, name="CheckArriveAtLocationTest"):
        """
        Setup actor and maximum allowed velovity
        """
        self._destination = destination
        self._distance = round(min_distance, 2)
        self._allow_exceed = allow_exceed
        self._actor = actor
        super(ArriveAtLocationTest, self).__init__(name, self._actor, self._distance, None, optional)

    def update(self):
        """
        Check location
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        location = CarlaDataProvider.get_location(self.actor)       

        extend_x = self.actor.bounding_box.extent.x if self.actor.bounding_box.extent is not None else 0
        distance_vector = self._destination - location
        distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
        self.actual_value  = distance - extend_x
        
        fv = self._actor.get_transform().rotation.get_forward_vector()
        if fv.x * distance_vector.x + fv.y * distance_vector.y < 0:
            is_exceed = True
        else:
            if distance < extend_x:
                is_exceed = True
            else:
                is_exceed = False

        if not is_exceed:
            if distance > extend_x + self._distance:
                self.test_status = "FAILURE"
            else:            
                self.test_status = "SUCCESS"        
        
        if self._allow_exceed and is_exceed:
            self.test_status = "SUCCESS"
        
        if not self._allow_exceed and is_exceed:
            self.test_status = "FAILURE"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status
    
    def terminate(self, new_status):
        if (self.test_status == "RUNNING") or (self.test_status == "INIT"):
            self.test_status = "FAILURE"
        return super().terminate(new_status)

class StandStillTimeTest(Criterion):

    """
    This class contains a standstill behavior of a scenario

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - duration: Duration of the behavior in seconds, true when less then expected duration.

    The condition terminates with SUCCESS, when the actor does not move
    """

    def __init__(self, actor, other_actor = None, duration=0, optional=False, name="CheckStandStillTimeTest"):
        """
        Setup actor
        """
        self._duration = duration
        self._actor = actor
        self._other_actor = other_actor
        self._other_actor_stay_in_safe_region = False
        self._actors_stay_in_same_lane = False
        self._lane_width = 3.5 # default width
        self._other_actor_run_dis = 0
        self._last_timestamp = 0 
        self._standstill_total_time = 0
        super(StandStillTimeTest, self).__init__(name, actor, self._duration, None, optional)

    def initialise(self):
        """
        Initialize the start time of this condition
        """
        self._start_time = GameTime.get_time()
        if self._other_actor is not None:
            self._other_actor_last_location = CarlaDataProvider.get_location(self._other_actor)
        else:
            self._actor_last_location = CarlaDataProvider.get_location(self._actor)

        super(StandStillTimeTest, self).initialise()

    def update(self):
        """
        Check if the _actor stands still (v=0)
        """
        new_status = py_trees.common.Status.RUNNING

        map = CarlaDataProvider.get_map()
        ego_wp =  map.get_waypoint(CarlaDataProvider.get_location(self._actor), project_to_road=False, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
        other_actor_wp = map.get_waypoint(CarlaDataProvider.get_location(self._other_actor)) if self._other_actor is not None else None

        if ego_wp is not None and other_actor_wp is not None:
            ego_actor_lane_id = ego_wp.lane_id
            other_actor_lane_id = other_actor_wp.lane_id
            if ego_actor_lane_id == other_actor_lane_id:
                self._lane_width = ego_wp.lane_width
                self._other_actor_run_dis += calculate_distance(CarlaDataProvider.get_location(self._other_actor), self._other_actor_last_location) 
        
        if self._other_actor is not None:
            self._other_actor_last_location = CarlaDataProvider.get_location(self._other_actor)
        else:
            actor_run_distance = calculate_distance(CarlaDataProvider.get_location(self._actor), self._actor_last_location)
            # print("run_dis = {}".format(actor_run_distance))
            if actor_run_distance > 2:
                self._other_actor_stay_in_safe_region = True

        if self._other_actor_run_dis > self._lane_width:
            self._other_actor_stay_in_safe_region = True

        velocity = CarlaDataProvider.get_velocity(self._actor)
        
        if velocity <= EPSILON and self._other_actor_stay_in_safe_region:
            accelate_time = round(GameTime.get_time() - self._start_time, 2)
            self._standstill_total_time += accelate_time
            self.actual_value = self._standstill_total_time
        else:
            if self.test_status != "SUCCESS" :
                if self.actual_value > 0: # keep still before
                    if self.actual_value <= self._duration:
                        self.test_status = "SUCCESS"
                    else:
                        self.test_status = "FAILURE" 
                else:
                    self.actual_value = 0

        self._start_time = GameTime.get_time()

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Terminate the criterion. Can be extended by the user-derived class
        """
        if self.test_status != "SUCCESS":
            self.test_status = "FAILURE"

        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class StandStillDistanceTest(Criterion):

    """
    This class contains an atomic test for maximum velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - distance: allow the actor keep min distance with the destination
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, destination, min_distance = 0.01, optional=False, name="CheckStandStillDistanceTest"):
        """
        Setup actor and maximum allowed velovity
        """
        self._destination = destination
        self._excepted_distance = round(min_distance, 2)
        self._actor = actor
        super(StandStillDistanceTest, self).__init__(name, self._actor, self._excepted_distance, None, optional)
    
    def initialise(self):
        """
        Initialize the start time of this condition
        """
        self._extend_x = self._actor.bounding_box.extent.x if self._actor.bounding_box.extent is not None else 0
        super(StandStillDistanceTest, self).initialise()
    
    def update(self):
        """
        Check location
        """
        new_status = py_trees.common.Status.RUNNING

        if self._actor is None:
            return new_status

        if self.test_status != "SUCCESS":
            location = CarlaDataProvider.get_location(self._actor) 
            velocity = CarlaDataProvider.get_velocity(self._actor)        
            distance_vector = self._destination - location
            distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
            if velocity <= EPSILON:                
                self.actual_value = distance - self._extend_x
                if distance > self._extend_x + self._excepted_distance:
                    self.test_status = "FAILURE"
                else:
                    self.test_status = "SUCCESS"
        
        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status
    
    def terminate(self, new_status):
        if (self.test_status == "RUNNING") or (self.test_status == "INIT"):
            self.test_status = "FAILURE"
        return super().terminate(new_status)
