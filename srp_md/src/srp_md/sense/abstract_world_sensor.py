from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class AbstractWorldSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = [str(i + 1) for i in range(7)]
        self._properties = {}
        for i in range(random.randint(2, 4)):
            self._properties[str(i + 1)] = [str(i + 1) for i in range(random.randint(1, 5))]
        self._RELATIONS = ["smaller", "equal", "bigger"]

        # Setup goal condition
        self._goal_prop = random.choice(list(self._properties.keys()))
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {}
            for prop in self._properties.keys():
                self._ass_prop[obj][prop] = random.choice(self._properties[prop])

    def process_data(self, data):
        consistent = False

        # Randomly choose objects from object list
        num_objs = random.randint(1, len(self._objs))
        objs = [srp_md.Var(name='X_{}'.format(i + 1), var_type="object", value=v, properties=self._ass_prop[v])
                for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)
        while not consistent:
            for relation in scene_graph.relations:
                var_i = objs[int(relation.name[relation.name.find('_', 1) + 1:relation.name.find('_', 2)]) - 1]
                var_j = objs[int(relation.name[relation.name.find('_', 2) + 1:]) - 1]
                if var_i.properties[self._goal_prop] < var_j.properties[self._goal_prop]:
                    relation.value = self._RELATIONS[0]
                elif var_i.properties[self._goal_prop] == var_j.properties[self._goal_prop]:
                    relation.value = self._RELATIONS[1]
                else:
                    relation.value = self._RELATIONS[2]
            consistent = scene_graph.check_consistency("abstract")

        self._logger.info('What are object names? %s', scene_graph.get_obj_names())
        self._logger.info('What are object values? %s', scene_graph.get_obj_values())
        self._logger.info('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.info('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.info('What are the objects property values? %s', scene_graph.get_prop_values(self._goal_prop))
        self._logger.info('Is this scene graph consistent? %s', consistent)

        return scene_graph


sense.sensors['abstract_world_sensor'] = AbstractWorldSensor
