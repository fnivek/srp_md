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
        self.goal_prop = "color"
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "material": random.choice(self._properties["material"])}

    def check_property(self, scene_graph, goal_prop):
        # If the goal property is None, then this is automatically true
        if goal_prop is None:
            return True

        # Initialize lists
        order_list = []
        bot_top = []

        # For each relation, do:
        for relation in scene_graph.relations:
            # Count how many relations that are 'on' or 'support' and add to order_list (from bot to top fashion)
            objs = relation.get_objs()
            obj_ids = [obj.id for obj in objs]
            if relation.value == 'on':
                obj_ids.reverse()
                order_list.append(obj_ids)
            elif relation.value == 'support':
                order_list.append(obj_ids)

        # If that count does not equal number of objects in the scene - 1, then False
        if len(order_list) != scene_graph.num_objs() - 1:
            return False

        # Build a list of variable index from bottom block to top-most block
        while len(order_list) != 0:
            # Save the size of order_list before going through for-loop
            before_size = len(order_list)
            # Add the order to the front or back of the bot_top list if one of the element matches
            for order in order_list:
                if bot_top == []:
                    bot_top += order
                elif bot_top[0] == order[-1]:
                    bot_top = order + bot_top[1:]
                elif bot_top[-1] == order[0]:
                    bot_top = bot_top[:-1] + order
                else:
                    continue
                # Pop off the element if it has been "used"
                order_list.pop(order_list.index(order))
            # Get the size after going through the for-loop
            after_size = len(order_list)
            # If there hasn't been any change (sign of infinite loop), then raise ValueError
            if before_size == after_size:
                raise ValueError()
        self._logger.debug('Checked Tower Order from Bottom to Top %s', bot_top)

        # Check if the blocks are ordered with desired property correctly, and if not, return False
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
            consistent = scene_graph.check_consistency("block")
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

    def process_data(self, demo_type, data):
        # Randomly choose objects from object list
        # num_objs = random.randint(3, len(self._objs))
        num_objs = 3
        objs = [srp_md.Object(id_num=i + 1, uuid=i + 1, assignment=self._ass_prop[v])
                for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)

        # Depending on the demo type wanted, input relations accordingly
        if demo_type == "only_goal":
            demo_graph = self.gen_goal_demo(scene_graph)
        elif demo_type == "only_not_goal":
            demo_graph = self.gen_not_goal_demo(scene_graph)
        elif demo_type == "random":
            demo_graph = random.choice([self.gen_goal_demo, self.gen_not_goal_demo])(scene_graph)

        # If error doesn't occur, check the scene graph's goal condition and print check up statements
        scene_graph = demo_graph
        goal_cond = self.check_property(scene_graph, self.goal_prop)

        self._logger.debug('What are object names? %s', scene_graph.get_obj_names())
        self._logger.debug('What are relation names? %s', scene_graph.get_rel_names())
        self._logger.debug('What are relation values? %s', scene_graph.get_rel_values())
        self._logger.debug('What are property values? %s', scene_graph.get_prop_values(self.goal_prop))
        self._logger.debug('Is this scene graph consistent? %s', scene_graph.check_consistency("block"))
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)

        return scene_graph


sense.sensors['block_tower_sensor'] = BlockTowerSensor
