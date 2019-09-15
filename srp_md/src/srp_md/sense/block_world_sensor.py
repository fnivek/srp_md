from __future__ import absolute_import
from builtins import str, range
import random
import logging

from . import sense
import srp_md


class BlockWorldSensor(sense.BaseSensor):
    def __init__(self):
        super(BlockWorldSensor, self).__init__()
        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Initialize basic info
        self._goal_type = "single stack order by color"
        self._objs = range(50)
        self.properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                           "material": ["metal", "wood", "plastic", "iron", "zinc"],
                           "letter": ["A", "B", "C", "D", "E"]}
        self._RELATIONS = ['disjoint', 'on', 'support', 'proximity']
        self.goal_prop = None
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self.properties["color"]),
                                   "material": random.choice(self.properties["material"]),
                                   "letter": random.choice(self.properties["letter"])}

    @property
    def min_num_objs(self):
        return self._min_num_objs

    @min_num_objs.setter
    def min_num_objs(self, num_objs):
        if num_objs < 3:
            raise ValueError('Must have at least 3 objects')
        self._min_num_objs = num_objs

    def single_stack(self, scene_graph, goal_prop, dir_config):
        # Decide which direction of the color spectrum blocks are stacked
        if dir_config == "bothway":
            direction = random.choice(["bot_left", "top_left"])
        elif dir_config == "oneway":
            direction = "top_left"
        else:
            self._logger.error('Please specify correct direction config.')
            raise ValueError()
        self._logger.debug('Which way is direction? %s', direction)

        # Get a list of integer ids for variables
        bot_top = list(range(1, scene_graph.num_objs() + 1))

        # Random shuffle if goal property is None
        if goal_prop is None:
            random.shuffle(bot_top)
        else:
            # Sort the objects based on goal property
            prop_list = list(self.properties[goal_prop])
            # If reversed direction, then reverse the property list order
            if direction == "bot_left":
                prop_list.reverse()
            prop_order = [prop_list.index(val) for val in scene_graph.get_prop_values(goal_prop)]
            bot_top = [var_id for _, var_id in sorted(zip(prop_order, bot_top))]

        # For each pair in bot_top list, set value for relation in correspondence
        for relation in scene_graph.relations:
            [id1, id2] = [obj.id for obj in relation.get_objs()]
            if bot_top.index(id1) < bot_top.index(id2):
                relation.value = "on"
            elif bot_top.index(id1) > bot_top.index(id2):
                relation.value = "support"
            else:
                self._logger.error('Something is terribly wrong!')

        self._logger.debug('Tower Order from Bottom to Top %s', bot_top)

        return scene_graph

    def single_stack_checker(self, scene_graph, goal_prop, dir_config):
        # Return True if only one object in the scene
        if scene_graph.num_objs() == 1:
            return True

        # Decide which direction of the color spectrum blocks are stacked
        if dir_config == "bothway":
            check_both = True
        elif dir_config == "oneway":
            check_both = False
        else:
            self._logger.error('Please specify correct direction config.')
            raise ValueError()

        # Keep a list of 0s and 1s to determine for each relation the order is reversed
        dir_list = [0] * scene_graph.num_relations()
        for ind in range(scene_graph.num_relations()):
            relation = scene_graph.relations[ind]
            # All relations must be support or on
            if relation.value not in ['on', 'support']:
                return False
            # Get the objects and the ordering of there properties
            [var_i, var_j] = relation.get_objs()
            prop_list = self.properties[goal_prop]
            prop_index_i = prop_list.index(var_i.assignment[goal_prop])
            prop_index_j = prop_list.index(var_j.assignment[goal_prop])
            # If i is earlier in the list it must support
            if prop_index_i < prop_index_j:
                if relation.value != "on":
                    dir_list[ind] = 1
            # If i is latter in the list it must be on
            elif prop_index_i > prop_index_j:
                if relation.value != "support":
                    dir_list[ind] = 1
            # else they have the same index and can be sorted in any order
            else:
                dir_list[ind] = 2

        # If all are 2, return True
        if all(x == 2 for x in dir_list):
            return True

        # Take out 2's which order doesn't matter
        dir_list = filter(lambda a: a != 2, dir_list)

        # If we need to check both directions, then see if all elements are same
        if check_both:
            first_ele = dir_list[0]
        # If we need to check only top_left case, then check if all entries are 0
        else:
            first_ele = 0

        if all(x == first_ele for x in dir_list):
            return True
        else:
            return False

    def check_goal(self, scene_graph):
        checking = True
        # If the goal type is None, then this is automatically true
        if self._goal_type is None:
            return checking

        elif self._goal_type == "single stack order by color":
            checking = self.single_stack_checker(scene_graph, "color", "oneway")

        elif self._goal_type == "single stack order by material":
            checking = self.single_stack_checker(scene_graph, "material", "oneway")

        elif self._goal_type == "single stack order by color both ways":
            checking = self.single_stack_checker(scene_graph, "color", "bothway")

        elif self._goal_type == "stacks by material order by color":
            checking = self.multiple_stack_checker(scene_graph, "oneway", sort="material",
                                                   order="color")

        elif self._goal_type == "stacks by random order by random":
            checking = self.multiple_stack_checker(scene_graph, "bothway", sort="random",
                                                   order="random")

        elif self._goal_type == "most stacks of 3":
            pass

        elif self._goal_type == "highest bipartite tower":
            pass

        elif self._goal_type == "single stack order by material":
            pass

        elif self._goal_type == "multiple stack":
            pass

        elif self._goal_type == "alone most unique, stack rest":
            pass

        elif self._goal_type == "alone most common, stack rest":
            pass

        return checking

    def gen_goal_demo(self, scene_graph):
        # Run appropriate goal generation function depending on the goal type
        if self._goal_type == "single stack order by color":
            scene_graph = self.single_stack(scene_graph, "color", "oneway")

        elif self._goal_type == "single stack order by material":
            scene_graph = self.single_stack(scene_graph, "material", "oneway")

        elif self._goal_type == "single stack order by color both ways":
            scene_graph = self.single_stack(scene_graph, "color", "bothway")

        elif self._goal_type == "stacks by material order by color":
            scene_graph = self.multiple_stack(scene_graph, "oneway", sort="material",
                                              order="color")

        elif self._goal_type == "stacks by random order by random":
            scene_graph = self.multiple_stack(scene_graph, "bothway", sort="random",
                                              order="random")

        elif self._goal_type == "most stacks of 3":
            pass

        elif self._goal_type == "highest bipartite tower":
            pass

        elif self._goal_type == "single stack order by material":
            pass

        elif self._goal_type == "multiple stack":
            pass

        elif self._goal_type == "alone most unique, stack rest":
            pass

        elif self._goal_type == "alone most common, stack rest":
            pass

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
                goal_cond = self.check_goal(scene_graph)
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
        goal_cond = self.check_goal(scene_graph)

        self._logger.debug('Objects: \n{}'.format(scene_graph.print_objs()))
        self._logger.debug('Relations: \n{}'.format(scene_graph.print_rels()))
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)

        return scene_graph


sense.sensors['block_world_sensor'] = BlockWorldSensor

goal_types = ["single stack order by color", "single stack order by material", "single stack order by color both ways",
              "stacks by material order by color", "stacks by random order by random",
              "most stacks of 3", "highest bipartite tower",
              "single stack order by material", "multiple stack",
              "alone most unique, stack rest", "alone most common, stack rest"]
sense.goal_types['block_world_sensor'] = goal_types

# A goal with objects that are not part of the goal
