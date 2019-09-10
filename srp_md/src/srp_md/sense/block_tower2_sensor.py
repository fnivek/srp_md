from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BlockTower2Sensor(sense.BaseSensor):
    def __init__(self):
        super(BlockTower2Sensor, self).__init__()
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

    def get_domain(self):
        return dict(self._properties)

    @property
    def min_num_objs(self):
        return self._min_num_objs

    @min_num_objs.setter
    def min_num_objs(self, num_objs):
        if num_objs < 3:
            raise ValueError('Must have at least 3 objects')
        self._min_num_objs = num_objs

    def check_property(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            # All relations must be support or on
            if relation.value not in ['support', 'on']:
                return False
            # Get the objects and the ordering of there properties
            [var_i, var_j] = relation.get_objs()
            prop_list = self._properties[goal_prop]
            prop_index_i = prop_list.index(var_i.assignment[goal_prop])
            prop_index_j = prop_list.index(var_j.assignment[goal_prop])
            # If i is earlier in the list it must support
            if prop_index_i < prop_index_j:
                if relation.value != "support":
                    return False
            # If i is latter in the list it must be on
            elif prop_index_i > prop_index_j:
                if relation.value != "on":
                    return False
            # else they have the same index and can be sorted in any order
        return True

    def gen_goal_demo(self, scene_graph):
        # Get a list of integer ids for variables
        bot_top = list(range(1, scene_graph.num_objs() + 1))

        # If goal property is set to None
        if self.goal_prop is None:
            # Just randomly shuffle the objects
            random.shuffle(bot_top)
        # If a goal property is given
        else:
            # Sort the objects based on goal property
            prop_list = self._properties[self.goal_prop]
            prop_order = [prop_list.index(val) for val in scene_graph.get_prop_values(self.goal_prop)]
            bot_top = [var_id for _, var_id in sorted(zip(prop_order, bot_top))]

        # For each pair in bot_top list, set value for relation in correspondence
        for relation in scene_graph.relations:
            [id1, id2] = [obj.id for obj in relation.get_objs()]
            if bot_top.index(id1) < bot_top.index(id2):
                relation.value = "support"
            elif bot_top.index(id1) > bot_top.index(id2):
                relation.value = "on"
            else:
                self._logger.error('Something is terribly wrong!')

        self._logger.debug('Tower Order from Bottom to Top %s', bot_top)

        return scene_graph

    def gen_not_goal_demo(self, scene_graph):
        # Initialize variables
        count = 0
        consistent = False
        goal_cond = True

        # While we get a demo that is either inconsistent or is a goal
        while not consistent or goal_cond:
            # Add to counter
            count += 1
            # Randomly set relation values
            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)
            # Check the consistency of this graph
            consistent = scene_graph.check_consistency("block2")
            # Pass if inconsistent
            if not consistent:
                pass
            # If consistent, check if this graph is goal
            else:
                goal_cond = self.check_property(scene_graph, self.goal_prop)
            # If iteration is over 100, then raise ValueError
            if count > 100:
                raise ValueError()

        self._logger.debug('What was the count? %s', count)

        return scene_graph

    def process_data(self, data):
        # Randomly choose objects from object list
        max_num_objs = min(len(self._objs), self._max_num_objs)
        num_objs = random.randint(self._min_num_objs, max_num_objs)
        objs = [srp_md.Object(id_num=i + 1, uuid=i + 1, assignment=self._ass_prop[v])
                for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)

        # Depending on the demo type wanted, input relations accordingly
        if self._demo_type == "only_goal":
            demo_graph = self.gen_goal_demo(scene_graph)
        elif self._demo_type == "only_not_goal":
            demo_graph = self.gen_not_goal_demo(scene_graph)
        elif self._demo_type == "random":
            demo_graph = random.choice([self.gen_goal_demo, self.gen_not_goal_demo])(scene_graph)

        # If error doesn't occur, check the scene graph's goal condition and print check up statements
        scene_graph = demo_graph
        goal_cond = self.check_property(scene_graph, self.goal_prop)

        self._logger.debug('What are object names? %s', scene_graph.get_obj_names())
        self._logger.debug('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.debug('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)

        return scene_graph


sense.sensors['block_tower2_sensor'] = BlockTower2Sensor
