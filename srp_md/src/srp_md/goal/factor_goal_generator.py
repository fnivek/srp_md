# Python
from __future__ import absolute_import
import logging
import sys
from random import choice
import itertools

# Ros
import rospy

# Project
from . import goal_generator
from srp_md.srv import GetGoal, GetGoalRequest
from srp_md.msg import Factor
import srp_md


class FactorGraphGoalGenerator(goal_generator.BaseGoalGenerator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._get_goal_client = None
        try:
            rospy.wait_for_service('get_goal', timeout=1)
            self._get_goal_client = rospy.ServiceProxy('get_goal', GetGoal)
        except rospy.ROSException as e:
            self._logger.error('Failed to get a client for /get_goal service: {}'.format(e))

    def generate_goal(self, factors, obs):
        self._logger.debug('Generating goal')

        if self._get_goal_client is None:
            self._logger.error('/get_goal service not avaliable cannot generate goal')
            return None

        self._logger.debug('Took factors %s', factors)

        # Fill in the request
        # TODO(Henry): Use the obs param to fill in the request (I made something for myself to test with feel free to
        #   change the data type)
        if obs is None:
            a = srp_md.Var('Up')
            a.properties['color'] = 'red'
            a.properties['spin'] = 'up'
            b = srp_md.Var('Down')
            b.properties['color'] = 'green'
            b.properties['spin'] = 'down'
            c = srp_md.Var('Strange')
            c.properties['color'] = 'blue'
            c.properties['spin'] = 'up'
            obs = srp_md.SceneGraph([a, b, c])
        req = GetGoalRequest()
        req.objects = obs.get_obj_names()
        # TODO(?): We need to figure out how to represent this
        req.classes = [
            choice([GetGoalRequest.CLASS_PROP, GetGoalRequest.CLASS_CONTAINER, GetGoalRequest.CLASS_SUPPORTER])
            for _ in req.objects
        ]
        # TODO(Kevin): If the objects always have one state then there is no need to have this (I leave it for now
        #              because it might be useful if we take noisy observations)
        # Objects will have one state because they have been observed and therfore are in one possible state
        req.num_states = [obj.num_states for obj in obs.objs]

        # Generate and fill in all factors
        #   For each type of factor generate all possible combinations
        #   If there are not enough vars for the factor the factor is skipped
        for factor_type, learned_factor in factors.iteritems():
            for objects in itertools.combinations(obs.objs, factor_type[0]):
                for pairs in itertools.combinations(obs.relations, factor_type[1]):
                    # Use the LearnedFactor to generate a ros_factor for each combination
                    ros_factor = learned_factor.gen_factor(objects + pairs).to_ros_factor()

            req.factors.append(ros_factor)

        self._logger.debug('Get goal request is:\n{}'.format(req))

        # Get the response
        resp = None
        try:
            resp = self._get_goal_client(req)
        except rospy.ServiceException as e:
            self._logger.error('Failed when calling /get_goal service: {}'.format(e))
            return None

        self._logger.debug('/get_goal response:\n{}'.format(resp))

        return resp


# Register the goal generator
goal_generator.goal_generators['factor_graph_goal_generator'] = FactorGraphGoalGenerator
