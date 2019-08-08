from __future__ import absolute_import
from builtins import range
from . import sense
import random
import logging


class BlockWorldSensor(sense.BaseSensor):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._objs = ["A", "B", "C", "D", "E"]
        self._predicates = ["On", "NextTo"]
        self._num_objs = 3

    def process_data(self, data):
        objs_copy = list(self._objs)
        random.shuffle(objs_copy)
        objects = objs_copy[:self._num_objs]
        num_predicates = random.choice([1, 2])
        predicates = [[random.choice(self._predicates), objects[i], objects[i + 1]] for i in range(num_predicates)]
        return (objects, predicates)


sense.sensors['block_world_sensor'] = BlockWorldSensor
