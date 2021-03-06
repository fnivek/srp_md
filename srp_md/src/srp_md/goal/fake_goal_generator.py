from __future__ import absolute_import
from . import goal_generator

import logging


class FakeGoalGenerator(goal_generator.BaseGoalGenerator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def generate_goal(self, factors=None):
        self._logger.debug('Generating goal')
        return ['fake_goal']


# Register the goal generator
goal_generator.goal_generators['fake_goal_generator'] = FakeGoalGenerator
