from __future__ import absolute_import
from builtins import str, range
from . import sense
import random
import logging


class BlockWorldSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C", "D", "E", "F", "G"]
        self._predicates = ["disjoint", "in", "on", "contain", "support", "proximity"]
        self._num_objs = random.randint(1, len(self._objs))

    def process_data(self, data):
        # Initialize scene graph with variable and predicate nodes
        self._scene_graph = {}
        self._variable_keys = ["X_" + str(i + 1) for i in range(self._num_objs)]
        self._predicate_keys = []
        for i in range(self._num_objs):
            for j in range(self._num_objs)[i:]:
                if i == j:
                    pass
                else:
                    self._predicate_keys.append("R_" + str(i + 1) + str(j + 1))

        # Randomly choose objects from object list
        objs_copy = list(self._objs)
        random.shuffle(objs_copy)
        for i in range(len(self._variable_keys)):
            self._scene_graph[self._variable_keys[i]] = objs_copy[i]

        # Randomly assign predicates
        for i in range(len(self._predicate_keys)):
            self._scene_graph[self._predicate_keys[i]] = random.choice(self._predicates)

        # Update the number of objects randomly
        self._num_objs = random.randint(1, len(self._objs))

        return self._scene_graph


sense.sensors['block_world_sensor'] = BlockWorldSensor
