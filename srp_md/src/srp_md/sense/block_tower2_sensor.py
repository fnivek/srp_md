from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BlockTower2Sensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C", "D", "E"]
        self._properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                            "material": ["metal", "wood", "plastic"]}
        self._RELATIONS = ['disjoint', 'on', 'support', 'proximity']
        self.goal_prop = "color"
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "material": random.choice(self._properties["material"])}

    def check_property(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            [var_i, var_j] = relation.get_objs()
            prop_list = self._properties[goal_prop]
            if prop_list.index(var_i.assignment[goal_prop]) <= prop_list.index(var_j.assignment[goal_prop]):
                if relation.value != "on":
                    return False
            elif prop_list.index(var_i.assignment[goal_prop]) > prop_list.index(var_j.assignment[goal_prop]):
                if relation.value != "support":
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
            objs = [srp_md.Object(id_num=i + 1, uuid=i + 1, assignment=self._ass_prop[v])
                    for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

            # Generate a consistent scene graph
            scene_graph = srp_md.SceneGraph(objs)

            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)
            goal_cond = self.check_property(scene_graph, self.goal_prop)
            if (demo_type == "only_goal") and (goal_cond):
                satisfied = True
            elif (demo_type == "only_not_goal") and (not goal_cond):
                satisfied = True
            elif demo_type == "random":
                satisfied = True
            consistent = scene_graph.check_consistency("block2")

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


sense.sensors['block_tower2_sensor'] = BlockTower2Sensor
