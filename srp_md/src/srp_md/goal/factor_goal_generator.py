# Python
from __future__ import absolute_import
import logging
import sys

# Ros
import rospy

# Project
from . import goal_generator
from srp_md.srv import GetGoal, GetGoalRequest


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

        req = GetGoalRequest()
        req.test.data = 'test'
        resp = None

        try:
            resp = self._get_goal_client(req)
        except rospy.ServiceException as e:
            self._logger.error('Failed when calling /get_goal service: {}'.format(e))
            return None

        self._logger.debug('/get_goal response: {}'.format(resp))

        return None


# Register the goal generator
goal_generator.goal_generators['factor_graph_goal_generator'] = FactorGraphGoalGenerator
