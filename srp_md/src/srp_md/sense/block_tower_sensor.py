from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BlockTowerSensor(sense.BaseSensor):
    def __init__(self):
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._objs = ["A", "B", "C", "D", "E"]
        self._properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                            "material": ["metal", "wood", "plastic"]}
        self._RELATIONS = ['disjoint', 'on', 'support', 'proximity']
        self._goal_prop = "color"
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "material": random.choice(self._properties["material"])}

    def check_property(self, scene_graph, goal_prop):
        order_list = []
        bot_top = []
        for relation in scene_graph.relations:
            # Count how many relations that are 'on' or 'support'
            if relation.value == 'on':
                var_ids = relation.return_objects()
                var_ids.reverse()
                order_list.append(var_ids)
            elif relation.value == 'support':
                var_ids = relation.return_objects()
                order_list.append(var_ids)

        # If that count does not equal number of objects in the scene - 1, then False
        if len(order_list) != len(scene_graph.objs) - 1:
            return False
        else:
            # Build a list of variable index from bottom block to top-most block
            while len(order_list) != 0:
                for order in order_list:
                    if bot_top == []:
                        bot_top += order
                    elif bot_top[0] == order[-1]:
                        bot_top = order + bot_top[1:]
                    elif bot_top[-1] == order[0]:
                        bot_top = bot_top[:-1] + order
                    else:
                        continue
                    order_list.pop(order_list.index(order))

            if goal_prop is None:
                return True
            else:
                prop_list = self._properties[goal_prop]
                prop_order = scene_graph.get_prop_values(goal_prop)
                for i in range(len(bot_top) - 1):
                    for relation in scene_graph.relations:
                        if prop_list.index(prop_order[bot_top[i] - 1]) > \
                                prop_list.index(prop_order[bot_top[i + 1] - 1]):
                            if relation.name == 'R_' + str(bot_top[i]) + '_' + str(bot_top[i + 1]):
                                if relation.value != "support":
                                    return False
                            elif relation.name == 'R_' + str(bot_top[i + 1]) + '_' + str(bot_top[i]):
                                if relation.value != "on":
                                    return False

        return True

    def gen_goal_demo(self, scene_graph):
        # Get a list of integer ids for variables and randomly shuffle them
        bot_top = list(range(1, len(scene_graph.objs) + 1))
        if self._goal_prop is None:
            random.shuffle(bot_top)
        else:
            prop_list = self._properties[self._goal_prop]
            prop_order = [prop_list.index(val) for val in scene_graph.get_prop_values(self._goal_prop)]
            bot_top = [var_id for _, var_id in sorted(zip(prop_order, bot_top))]

        # Initialize all relations to be "disjoint"
        for relation in scene_graph.relations:
            relation.value = "disjoint"

        # For each pair in bot_top list, set value for relation in correspondence
        for i in range(len(bot_top) - 1):
            for relation in scene_graph.relations:
                if relation.name == 'R_' + str(bot_top[i]) + '_' + str(bot_top[i + 1]):
                    relation.value = "support"
                elif relation.name == 'R_' + str(bot_top[i + 1]) + '_' + str(bot_top[i]):
                    relation.value = "on"

        self._logger.debug('Tower Order from Bottom to Top %s', bot_top)

        return scene_graph

    def gen_not_goal_demo(self, scene_graph):
        count = 0
        consistent = False
        goal_cond = True

        while not consistent or goal_cond:
            count += 1

            for relation in scene_graph.relations:
                relation.value = random.choice(self._RELATIONS)

            consistent = scene_graph.check_consistency("block")
            if not consistent:
                pass
            else:
                goal_cond = self.check_property(scene_graph, self._goal_prop)

            if count > 100:
                self._logger.warning('Desired scene graph could not be generated during given time. \
                    Please retry!')
                break

        self._logger.debug('What was the count? %s', count)

        return scene_graph

    def process_data(self, demo_type, data):
        # Randomly choose objects from object list
        num_objs = random.randint(3, len(self._objs))
        objs = [srp_md.Var(name='X_{}'.format(i + 1), var_type="object", value=v, properties=self._ass_prop[v])
                for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)

        # If not consistent, early quit and do next iteration
        if demo_type == "only_goal":
            scene_graph = self.gen_goal_demo(scene_graph)
        elif demo_type == "only_not_goal":
            scene_graph = self.gen_not_goal_demo(scene_graph)
        elif demo_type == "random":
            scene_graph = random.choice([self.gen_goal_demo, self.gen_not_goal_demo])(scene_graph)
        goal_cond = self.check_property(scene_graph, self._goal_prop)

        self._logger.debug('What are object names? %s', scene_graph.get_obj_names())
        self._logger.debug('What are object values? %s', scene_graph.get_obj_values())
        self._logger.debug('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.debug('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.debug('What are property values? %s', scene_graph.get_prop_values(self._goal_prop))
        self._logger.debug('Is this scene graph consistent? %s', scene_graph.check_consistency("block"))
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)

        return scene_graph


sense.sensors['block_tower_sensor'] = BlockTowerSensor
