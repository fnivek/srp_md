from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class AbstractWorldSensor(sense.BaseSensor):
    def __init__(self):
        super(AbstractWorldSensor, self).__init__()
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = [str(i + 1) for i in range(7)]
        self._properties = {}
        for i in range(random.randint(2, 4)):
            self._properties[str(i + 1)] = [str(i + 1) for i in range(random.randint(1, 5))]
        self._RELATIONS = ["smaller", "equal", "bigger"]

        # Setup goal condition
        self.goal_prop = random.choice(list(self._properties.keys()))
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {}
            for prop in self._properties.keys():
                self._ass_prop[obj][prop] = random.choice(self._properties[prop])

    def check_property(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            [var_i, var_j] = relation.get_objs()

            if var_i.assignment[goal_prop] < var_j.assignment[goal_prop]:
                if relation.value != self._RELATIONS[0]:
                    return False
            elif var_i.assignment[goal_prop] == var_j.assignment[goal_prop]:
                if relation.value != self._RELATIONS[1]:
                    return False
            else:
                if relation.value != self._RELATIONS[2]:
                    return False
        return True

    def process_data(self, data):
        satisfied = False
        consistent = False
        count = 0

        while (not satisfied or not consistent):
            count += 1

            # Randomly choose objects from object list
            num_objs = random.randint(1, len(self._objs))
            objs = [srp_md.Object(id_num=i + 1, uuid=i + 1, assignment=self._ass_prop[v])
                    for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

            # Generate a consistent scene graph
            scene_graph = srp_md.SceneGraph(objs)

            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)
            goal_cond = self.check_property(scene_graph, self.goal_prop)
            if (self._demo_type == "only_goal") and (goal_cond):
                satisfied = True
            elif (self._demo_type == "only_not_goal") and (not goal_cond):
                satisfied = True
            elif self._demo_type == "random":
                satisfied = True
            consistent = scene_graph.check_consistency("abstract")

            if count > 100:
                self._logger.warning('Desired scene graph could not be generated during given time. \
                    Please retry!')
                break

        self._logger.debug('What are object names? %s', scene_graph.get_obj_names())
        self._logger.debug('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.debug('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.debug('Is this scene graph consistent? %s', consistent)
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)
        self._logger.debug('What was the count? %s', count)

        return scene_graph


sense.sensors['abstract_world_sensor'] = AbstractWorldSensor

goal_types = ["single line by number"]
sense.goal_types['abstract_world_sensor'] = goal_types
