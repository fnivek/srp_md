from __future__ import absolute_import
from builtins import str
from builtins import range
from . import sense
import srp_md

import logging
from random import choice, shuffle, randint
import itertools


class CanTowerSensor(sense.BaseSensor):
    """ Sensor that gives examples of a can tower.

    This sensor will produce positive and negative examples of the goal, a tower of n1 cans in can world. Can world has
    n2 locations n3 cans and n4 distractor objects. There is one relational predicate ON(X, Y). The goal is for the
    goal_height cans to be in a tower or in formal logic ON(Z, Y), ON(Y, X), where X, Y, and Z are different possible
    cans.

    """
    def __init__(self, num_cans=2, goal_height=2, num_locations=2, num_distractor_objs=0):
        super(CanTowerSensor, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._goal_height = goal_height
        self._cans = ['c' + str(x) for x in range(num_cans)]
        self._locs = ['l' + str(x) for x in range(num_locations)]
        self._distractor_objs = ['d' + str(x) for x in range(num_distractor_objs)]

        # Build the full space of predicates
        self._predicates = [
            'ON({}, {})'.format(x, y)
            for x, y in itertools.permutations(self._cans + self._locs + self._distractor_objs, 2)
            if x not in self._locs
        ]
        self._logger.debug('Predicate space is \n\t%s', str(self._predicates))

    def preds_to_bin_vec(self, preds):
        return [True if pred in preds else False for pred in self._predicates]

    def process_data(self, data):
        to_place = []
        cans = list(self._cans)
        positive = False

        # Positive example
        if choice([True, False]):
            self._logger.debug('Making a positive example of can tower')
            positive = True
            can_tower = srp_md.reservoir_sample(self._cans, self._goal_height)
            to_place.append(can_tower)
            for can in can_tower:
                cans.remove(can)
            self._logger.debug('Can tower is: %s', str(can_tower))

        to_place.extend(srp_md.reservoir_sample(self._distractor_objs, randint(0, len(self._distractor_objs))))
        to_place.extend(srp_md.reservoir_sample(cans, randint(0, randint(0, len(cans)))))
        shuffle(to_place)
        self._logger.debug('Items to be placed in this order: %s', str(to_place))

        # Place the items and watch to see if positive example made by chance
        predicates = []
        tower_height = {l: 0 for l in self._locs}
        tops = {l: l for l in self._locs}
        for item in to_place:
            loc = choice(self._locs)
            # Stack all items in a list in a row at the same location
            if not hasattr(item, '__iter__'):
                item = [item]

            for sub_item in item:
                predicates.append('ON({}, {})'.format(sub_item, tops[loc]))
                tops[loc] = sub_item
                # Keep track of tower hieght
                if sub_item[0] == 'c':
                    tower_height[loc] += 1
                    if tower_height[loc] >= self._goal_height:
                        self._logger.debug('Made a positive example')
                        positive = True
                else:
                    tower_height[loc] = 0

        self._logger.debug('%s example with predicates %s', str(positive), str(predicates))
        return [self.preds_to_bin_vec(predicates), positive]


sense.sensors['can_tower_sensor'] = CanTowerSensor
