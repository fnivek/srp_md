# Python
from __future__ import absolute_import
import logging

# Ros

# Project
from . import goal_evaluator


class AdaptGoalEvaluator(goal_evaluator.BaseGoalEvaluator):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._sense_category = [['fake_sensor'], ['example_sensor', 'can_tower_sensor'],
                                ['posecnn_sensor', 'block_world_sensor', 'pen_world_sensor',
                                 'book_world_sensor', 'abstract_world_sensor']]

    def evaluate_goal(self, current_sensor, sensor_name, goal_instance):
        evaluation = True

        if sensor_name in self._sense_category[0]:
            self._logger.debug('Please add measure of evaluation for fake sensor')
            evaluation = None

        elif sensor_name in self._sense_category[1]:
            self._logger.debug('Please add measure of evaluation for version space sensors')
            evaluation = None

        elif sensor_name in self._sense_category[2]:
            if sensor_name == 'block_world_sensor':
                if not goal_instance.check_consistency("block"):
                    evaluation = False

            elif sensor_name in ['pen_world_sensor', 'book_world_sensor']:
                des_prop = "color"
                for relation in goal_instance.relations:
                    var_ids = relation.return_objects()
                    var_i = goal_instance.objs[var_ids[0] - 1]
                    var_j = goal_instance.objs[var_ids[1] - 1]
                    prop_list = current_sensor._properties[des_prop]
                    if prop_list.index(var_i.properties[des_prop]) <= prop_list.index(var_j.properties[des_prop]):
                        if relation.value != "left":
                            evaluation = False
                    elif prop_list.index(var_i.properties[des_prop]) > prop_list.index(var_j.properties[des_prop]):
                        if relation.value != "right":
                            evaluation = False
                if not goal_instance.check_consistency("pen"):
                    evaluation = False

            elif sensor_name == 'abstract_world_sensor':
                des_prop = current_sensor._goal_prop
                for relation in goal_instance.relations:
                    var_ids = relation.return_objects()
                    var_i = goal_instance.objs[var_ids[0] - 1]
                    var_j = goal_instance.objs[var_ids[1] - 1]
                    if var_i.properties[des_prop] < var_j.properties[des_prop]:
                        if relation.value != current_sensor._RELATIONS[0]:
                            evaluation = False
                    elif var_i.properties[des_prop] == var_j.properties[des_prop]:
                        if relation.value != current_sensor._RELATIONS[1]:
                            evaluation = False
                    else:
                        if relation.value != current_sensor._RELATIONS[2]:
                            evaluation = False
                if not goal_instance.check_consistency("abstract"):
                    evaluation = False

            else:
                self._logger.debug('Please add measure of evaluation for this factor graph sensor: %s', sensor_name)

        else:
            self._logger.debug('Please add measure of evaluation for sensor: %s', sensor_name)

        if evaluation:
            self._logger.info('This is a goal instance in the world %s', sensor_name)
            return evaluation
        elif evaluation is None:
            self._logger.info('Evaluation cannot be processed')
            return evaluation
        else:
            self._logger.info('This is not a goal instance in the world %s', sensor_name)
            return evaluation


# Register the goal generator
goal_evaluator.goal_evaluators['adapt_goal_evaluator'] = AdaptGoalEvaluator
