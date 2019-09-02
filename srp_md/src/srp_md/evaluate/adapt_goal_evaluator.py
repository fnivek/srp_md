# Python
from __future__ import absolute_import
import logging

# Ros

# Project
from . import goal_evaluator
# from ..sense import *


class AdaptGoalEvaluator(goal_evaluator.BaseGoalEvaluator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._sense_category = [['fake_sensor'], ['example_sensor', 'can_tower_sensor'],
                                ['posecnn_sensor', 'block_world_sensor', 'pen_world_sensor',
                                 'book_world_sensor', 'abstract_world_sensor', 'block_tower_sensor']]

    def evaluate_goal(self, current_sensor, sensor_name, goal_instance):
        evaluation = True

        if sensor_name in self._sense_category[0]:
            self._logger.debug('Please add measure of evaluation for fake sensor')
            evaluation = None

        elif sensor_name in self._sense_category[1]:
            self._logger.debug('Please add measure of evaluation for version space sensors')
            evaluation = None

        elif sensor_name in self._sense_category[2]:
            if sensor_name in ['block_world_sensor', 'block_tower_sensor']:
                evaluation = goal_instance.check_consistency("block")
                if sensor_name == 'block_tower_sensor':
                    evaluation = goal_instance.check_consistency("block") and \
                        current_sensor.check_property(goal_instance, current_sensor.goal_prop)

            elif sensor_name in ['pen_world_sensor', 'book_world_sensor']:
                evaluation = goal_instance.check_consistency("pen") and \
                    current_sensor.check_property(goal_instance, current_sensor.goal_prop)

            elif sensor_name == 'abstract_world_sensor':
                evaluation = goal_instance.check_consistency("abstract") and \
                    current_sensor.check_property(goal_instance, current_sensor.goal_prop)

            else:
                self._logger.debug('Please add measure of evaluation for this factor graph sensor: %s', sensor_name)

        else:
            self._logger.debug('Please add measure of evaluation for sensor: %s', sensor_name)

        if evaluation:
            self._logger.debug('This is a goal instance in the world %s', sensor_name)
        elif evaluation is None:
            self._logger.debug('Evaluation cannot be processed')
        else:
            self._logger.debug('This is not a goal instance in the world %s', sensor_name)
        return evaluation


# Register the goal generator
goal_evaluator.goal_evaluators['adapt_goal_evaluator'] = AdaptGoalEvaluator
