import sense
import srp_md

import logging
from random import choice, shuffle, randint


class CanTowerSensor(sense.BaseSensor):
    """ Sensor that gives examples of a can tower.

    This sensor will produce positive and negative examples of the goal, a tower of n1 cans in can world. Can world has
    n2 locations n3 cans and n4 distractor objects. There is one relational predicate ON(X, Y). The goal is for the
    goal_height cans to be in a tower or in formal logic ON(Z, Y), ON(Y, X), where X, Y, and Z are different possible
    cans.

    """
    def __init__(self, num_cans=5, goal_height=3, num_locations=3, num_distractor_objs=5):
        super(CanTowerSensor, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._goal_height = goal_height
        self._cans = ['c' + str(x) for x in xrange(num_cans)]
        self._locs = ['l' + str(x) for x in xrange(num_locations)]
        self._distractor_objs = ['d' + str(x) for x in xrange(num_distractor_objs)]

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
        return [predicates, positive]


sense.sensors['can_tower_sensor'] = CanTowerSensor
