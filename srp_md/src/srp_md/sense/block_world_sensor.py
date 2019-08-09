from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BlockWorldSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C"]

    def process_data(self, data):
        # Randomly choose objects from object list
        num_objs = random.randint(1, len(self._objs))
        objs = [srp_md.Var(name='X_{}'.format(i), value=v) for i, v in
                enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate the scene graph
        scene_graph = srp_md.SceneGraph(objs)
        for relation in scene_graph.relations:
            relation.value = random.choice(srp_md.SceneGraph.RELATIONS)

        return scene_graph


sense.sensors['block_world_sensor'] = BlockWorldSensor
