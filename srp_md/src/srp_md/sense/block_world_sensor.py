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
        self._objs = range(5)
        self._properties = {"color": ["red", "orange", "yellow", "green", "blue", "indigo", "purple"],
                            "material": ["metal", "wood", "plastic", "iron", "zinc"],
                            "letter": ["A", "B", "C", "D", "E"]}
        self._RELATIONS = ['disjoint', 'on', 'support', 'proximity']
        self.goal_prop = None
        self._ass_prop = {}
        for obj in self._objs:
            self._ass_prop[obj] = {"color": random.choice(self._properties["color"]),
                                   "material": random.choice(self._properties["material"]),
                                   "letter": random.choice(self._properties["letter"])}

    def single_stack(self, scene_graph, goal_prop):
        # Get a list of integer ids for variables
        bot_top = list(range(1, scene_graph.num_objs() + 1))

        # Random shuffle if goal property is None
        if goal_prop is None:
            random.shuffle(bot_top)
        else:
            # Sort the objects based on goal property
            prop_list = self._properties[goal_prop]
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

    def single_stack_checker(self, scene_graph, goal_prop):
        for relation in scene_graph.relations:
            # All relations must be support or on
            if relation.value not in ['on', 'support']:
                return False
            # Get the objects and the ordering of there properties
            [var_i, var_j] = relation.get_objs()
            prop_list = self._properties[goal_prop]
            prop_index_i = prop_list.index(var_i.assignment[goal_prop])
            prop_index_j = prop_list.index(var_j.assignment[goal_prop])
            # If i is earlier in the list it must support
            if prop_index_i < prop_index_j:
                if relation.value != "on":
                    return False
            # If i is latter in the list it must be on
            elif prop_index_i > prop_index_j:
                if relation.value != "support":
                    return False
            # else they have the same index and can be sorted in any order
        return True

    def check_goal(self, scene_graph, goal_type):
        checking = True
        # If the goal type is None, then this is automatically true
        if goal_type is None:
            return checking

        elif goal_type == "single stack order by color":
            checking = self.single_stack_checker(scene_graph, "color")

        elif goal_type == "single stack order by material":
            checking = self.single_stack_checker(scene_graph, "material")

        elif goal_type == "single stack order by color both ways":
            pass

        elif goal_type == "stacks by material order by color":
            pass

        elif goal_type == "stacks by random order by random":
            pass

        elif goal_type == "most stacks of 3":
            pass

        elif goal_type == "highest bipartite tower":
            pass

        elif goal_type == "single stack order by material":
            pass

        elif goal_type == "multiple stack":
            pass

        elif goal_type == "alone most unique, stack rest":
            pass

        elif goal_type == "alone most common, stack rest":
            pass

        return checking

    def gen_goal_demo(self, scene_graph, goal_type):
        # Run appropriate goal generation function depending on the goal type
        if goal_type == "single stack order by color":
            scene_graph = self.single_stack(scene_graph, "color")

        elif goal_type == "single stack order by material":
            scene_graph = self.single_stack(scene_graph, "material")

        elif goal_type == "single stack order by color both ways":
            pass

        elif goal_type == "stacks by material order by color":
            pass

        elif goal_type == "stacks by random order by random":
            pass

        elif goal_type == "most stacks of 3":
            pass

        elif goal_type == "highest bipartite tower":
            pass

        elif goal_type == "single stack order by material":
            pass

        elif goal_type == "multiple stack":
            pass

        elif goal_type == "alone most unique, stack rest":
            pass

        elif goal_type == "alone most common, stack rest":
            pass

        return scene_graph

    def gen_not_goal_demo(self, scene_graph, goal_type):
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
                goal_cond = self.check_goal(scene_graph, goal_type)
            # If iteration is over 100, then raise ValueError
            if count > 100:
                raise ValueError()

        self._logger.debug('What was the count? %s', count)

        return scene_graph

    def process_data(self, data):
        # Randomly choose objects from object list
        num_objs = random.randint(self._min_num_objs, len(self._objs))
        objs = [srp_md.Object(id_num=i + 1, uuid=i + 1, assignment=self._ass_prop[v])
                for i, v in enumerate(srp_md.reservoir_sample(self._objs, num_objs))]

        # Generate a consistent scene graph
        scene_graph = srp_md.SceneGraph(objs)

        # Depending on the demo type wanted, input relations accordingly
        if self._demo_type == "only_goal":
            demo_graph = self.gen_goal_demo(scene_graph, self._goal_type)
        elif self._demo_type == "only_not_goal":
            demo_graph = self.gen_not_goal_demo(scene_graph, self._goal_type)
        elif self._demo_type == "random":
            demo_graph = random.choice([self.gen_goal_demo, self.gen_not_goal_demo])(scene_graph, self._goal_type)

        # If error doesn't occur, check the scene graph's goal condition and print check up statements
        scene_graph = demo_graph
        goal_cond = self.check_goal(scene_graph, self._goal_type)

        self._logger.debug('Objects: \n{}'.format(scene_graph.print_objs()))
        self._logger.debug('Relations: \n{}'.format(scene_graph.print_rels()))
        self._logger.debug('Is goal condition satisfied? %s', goal_cond)

        return scene_graph


sense.sensors['block_world_sensor'] = BlockWorldSensor

goal_types = ["single stack order by color", "single stack order by material", "single stack order by color both ways"
              "stacks by material order by color", "stacks by random order by random",
              "most stacks of 3", "highest bipartite tower",
              "single stack order by material", "multiple stack",
              "alone most unique, stack rest", "alone most common, stack rest"]
sense.goal_types['block_world_sensor'] = goal_types

# A goal with objects that are not part of the goal
