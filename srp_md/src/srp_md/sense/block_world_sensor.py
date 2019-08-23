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
        self._RELATIONS = ['disjoint', 'on', 'support', 'proximity']

    def process_data(self, demo_type, data):
        satisfied = False

        # Randomly choose objects from object list
        num_objs = random.randint(1, len(self._objs))
        objs = [srp_md.Var(name='X_{}'.format(i + 1), var_type="object", value=v) for i, v in
                enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)
        while not satisfied:
            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)
            consistent = scene_graph.check_consistency("block")

            # Set end condition for while loop depending on demo type we want
            if (demo_type == "only_goal") and (consistent):
                satisfied = True
            elif (demo_type == "only_not_goal") and (not consistent):
                satisfied = True
            elif demo_type == "random":
                satisfied = True

        self._logger.info('What are object names? %s', scene_graph.get_obj_names())
        self._logger.info('What are object values? %s', scene_graph.get_obj_values())
        self._logger.info('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.info('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.info('Is this scene graph consistent? %s', consistent)
        self._logger.info('Is goal condition satisfied? %s', satisfied)

        return scene_graph


sense.sensors['block_world_sensor'] = BlockWorldSensor
