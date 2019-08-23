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

    def check_property(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            var_ids = relation.return_objects()
            var_i = scene_graph.objs[var_ids[0] - 1]
            var_j = scene_graph.objs[var_ids[1] - 1]

            if var_i.properties[goal_prop] < var_j.properties[goal_prop]:
                if relation.value != self._RELATIONS[0]:
                    return False
            elif var_i.properties[goal_prop] == var_j.properties[goal_prop]:
                if relation.value != self._RELATIONS[1]:
                    return False
            else:
                if relation.value != self._RELATIONS[2]:
                    return False
        return True

    def process_data(self, demo_type, data):
        satisfied = False
        consistent = False
        count = 0

        while (not satisfied or not consistent):
            count += 1

            # Randomly choose objects from object list
            num_objs = random.randint(1, len(self._objs))
            objs = [srp_md.Var(name='X_{}'.format(i + 1), var_type="object", value=v, properties=self._ass_prop[v])
                    for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

            # Generate a consistent scene graph
            scene_graph = srp_md.SceneGraph(objs)

            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)
            goal_cond = self.check_property(scene_graph, self._goal_prop)
            if (demo_type == "only_goal") and (goal_cond):
                satisfied = True
            elif (demo_type == "only_not_goal") and (not goal_cond):
                satisfied = True
            elif demo_type == "random":
                satisfied = True
            consistent = scene_graph.check_consistency("abstract")

            if count > 100:
                self._logger.warning('Desired scene graph could not be generated during given time. \
                    Please retry!')
                break

        self._logger.info('What are object names? %s', scene_graph.get_obj_names())
        self._logger.info('What are object values? %s', scene_graph.get_obj_values())
        self._logger.info('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.info('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.info('Is this scene graph consistent? %s', consistent)
        self._logger.info('Is goal condition satisfied? %s', goal_cond)
        self._logger.info('What was the count? %s', count)

        return scene_graph


sense.sensors['abstract_world_sensor'] = AbstractWorldSensor
