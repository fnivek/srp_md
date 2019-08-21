# Python
from __future__ import absolute_import
import logging

# Ros

# Project
from . import goal_evaluator


class AdaptGoalEvaluator(goal_evaluator.BaseGoalEvaluator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def evaluate_goal(self, goal_instance):
        self._logger.debug('The goal instance is %s', goal_instance)
        return None


# Register the goal generator
goal_evaluator.goal_evaluators['adapt_goal_evaluator'] = AdaptGoalEvaluator
