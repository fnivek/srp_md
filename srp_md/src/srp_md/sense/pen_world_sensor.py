from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class PenWorldSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C", "D", "E"]
        self._properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                            "type": ["pen", "pencil"],
                            "material": ["plastic", "metal", "wood"],
                            "shape": ["round", "sharp"]}
        self._RELATIONS = ["left", "right"]
        self._goal_prop = "color"
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "type": random.choice(self._properties["type"]),
                                   "material": random.choice(self._properties["material"]),
                                   "shape": random.choice(self._properties["shape"])}

    def check_property(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            var_ids = relation.return_objects()
            var_i = scene_graph.objs[var_ids[0] - 1]
            var_j = scene_graph.objs[var_ids[1] - 1]
            prop_list = self._properties[goal_prop]
            if prop_list.index(var_i.properties[goal_prop]) <= prop_list.index(var_j.properties[goal_prop]):
                if relation.properties['value'] != "left":
                    return False
            elif prop_list.index(var_i.properties[goal_prop]) > prop_list.index(var_j.properties[goal_prop]):
                if relation.properties['value'] != "right":
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
                relation.properties['value'] = random.choice(self._RELATIONS)
            goal_cond = self.check_property(scene_graph, self._goal_prop)
            if (demo_type == "only_goal") and (goal_cond):
                satisfied = True
            elif (demo_type == "only_not_goal") and (not goal_cond):
                satisfied = True
            elif demo_type == "random":
                satisfied = True
            consistent = scene_graph.check_consistency("pen")

            if count > 100:
                self._logger.warning('Desired scene graph could not be generated during given time. \
                    Please retry!')
                break

        self._logger.debug('What are object names? %s', scene_graph.get_obj_names())
        self._logger.debug('What are object values? %s', scene_graph.get_obj_values())
        self._logger.debug('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.debug('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.debug('Is this scene graph consistent? %s', consistent)
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)
        self._logger.debug('What was the count? %s', count)

        return scene_graph


sense.sensors['pen_world_sensor'] = PenWorldSensor
