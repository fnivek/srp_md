from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BookWorldSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P"]
        self._properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                            "genre": ["mystery", "sci-fi", "romance", "non-fiction"],
                            "author": ["Kevin French", "Henry Kim", "Jana Pavlasek", "Zhen Zeng"],
                            "cover": ["paper", "wood"],
                            "size": ["small", "medium", "large"]}
        self._RELATIONS = ["left", "right"]
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "genre": random.choice(self._properties["genre"]),
                                   "author": random.choice(self._properties["author"]),
                                   "cover": random.choice(self._properties["cover"]),
                                   "size": random.choice(self._properties["size"])}

    def process_data(self, data):
        consistent = False

        # Randomly choose objects from object list
        num_objs = random.randint(1, 8)
        objs = [srp_md.Var(name='X_{}'.format(i + 1), value=v, properties=self._ass_prop[v]) for i, v in
                enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)
        while not consistent:
            for relation in scene_graph.relations:
                var_i = objs[int(relation.name[relation.name.find('_', 1) + 1:relation.name.find('_', 2)]) - 1]
                var_j = objs[int(relation.name[relation.name.find('_', 2) + 1:]) - 1]
                if (self._properties["color"].index(var_i.properties["color"]) <=
                        self._properties["color"].index(var_j.properties["color"])):
                    relation.value = "left"
                else:
                    relation.value = "right"
            consistent = scene_graph.check_consistency("pen")

        self._logger.info('What are object names? %s', scene_graph.get_obj_names())
        self._logger.info('What are object values? %s', scene_graph.get_obj_values())
        self._logger.info('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.info('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.info('Is this scene graph consistent? %s', consistent)

        return scene_graph


sense.sensors['book_world_sensor'] = BookWorldSensor
