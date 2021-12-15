#!/usr/bin/env python

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (AtomicBehavior, get_actor_control)

class DecelerateToVelocity(AtomicBehavior):

    """
    This class contains an atomic acceleration behavior. The controlled
    traffic participant will accelerate with _throttle_value_ until reaching
    a given _target_velocity_

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - throttle_value: The amount of throttle used to accelerate in [0,1]
    - target_velocity: The target velocity the actor should reach in m/s

    The behavior will terminate, if the actor's velocity is at least target_velocity
    """

    def __init__(self, actor, brake_value, target_velocity, name="Deceleration"):
        """
        Setup parameters including acceleration value (via throttle_value)
        and target velocity
        """
        super(DecelerateToVelocity, self).__init__(name, actor)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._control, self._type = get_actor_control(actor)
        self._brake_value = brake_value
        self._target_velocity = target_velocity

    def initialise(self):
        # In case of walkers, we have to extract the current heading
        if self._type == 'walker':
            self._control.speed = self._target_velocity
            self._control.direction = CarlaDataProvider.get_transform(self._actor).get_forward_vector()

        super(DecelerateToVelocity, self).initialise()

    def update(self):
        """
        Set throttle to throttle_value, as long as velocity is < target_velocity
        """
        new_status = py_trees.common.Status.RUNNING

        if self._type == 'vehicle':
            if CarlaDataProvider.get_velocity(self._actor) > self._target_velocity:
                self._control.brake  = self._brake_value
            else:
                new_status = py_trees.common.Status.SUCCESS
                self._control.brake  = 0

        self._actor.apply_control(self._control)
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status