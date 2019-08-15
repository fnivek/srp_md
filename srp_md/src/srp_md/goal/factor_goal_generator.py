from __future__ import absolute_import
from . import goal_generator

import logging


class FactorGraphGoalGenerator(goal_generator.BaseGoalGenerator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def generate_goal(self, factors):
        self._logger.debug('Generating goal')
        self._logger.debug('Took factors %s', factors)
        return None


# Register the goal generator
goal_generator.goal_generators['factor_graph_goal_generator'] = FactorGraphGoalGenerator
