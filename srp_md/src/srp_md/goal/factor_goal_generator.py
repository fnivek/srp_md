# Python
from __future__ import absolute_import
import logging
import sys
from random import choice

# Ros
import rospy

# Project
from . import goal_generator
from srp_md.srv import GetGoal, GetGoalRequest
from srp_md.msg import Factor


class FactorGraphGoalGenerator(goal_generator.BaseGoalGenerator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._get_goal_client = None
        try:
            rospy.wait_for_service('get_goal', timeout=1)
            self._get_goal_client = rospy.ServiceProxy('get_goal', GetGoal)
        except rospy.ROSException as e:
            self._logger.error('Failed to get a client for /get_goal service: {}'.format(e))

    def generate_goal(self, factors):
        if self._get_goal_client is None:
            self._logger.error('/get_goal service not avaliable cannot generate goal')
            return None

        self._logger.debug('Generating goal')
        self._logger.debug('Took factors %s', factors)

        # Fill in the request
        req = GetGoalRequest()
        req.objects = ['A', 'B', 'C']
        req.classes = [
            choice([GetGoalRequest.CLASS_PROP, GetGoalRequest.CLASS_CONTAINER, GetGoalRequest.CLASS_SUPPORTER])
            for _ in range(len(req.objects))
        ]
        for factor_type, value in factors.iteritems():
            ros_factor = Factor()
            ros_factor.num_objs = factor_type[0]
            ros_factor.num_relations = factor_type[1]
            # TODO(Kevin): Fill this in correctly, they must be in a particular order that is known to here and in cpp
            ros_factor.probs = [val for val in value.values()]
            req.factors.append(ros_factor)

        # Get the response
        resp = None
        try:
            resp = self._get_goal_client(req)
        except rospy.ServiceException as e:
            self._logger.error('Failed when calling /get_goal service: {}'.format(e))
            return None

        self._logger.debug('/get_goal response:\n{}'.format(resp))

        return ['factor', 'graph', 'goal']


# Register the goal generator
goal_generator.goal_generators['factor_graph_goal_generator'] = FactorGraphGoalGenerator
