""" Act

Contains the robot acting utilities for srp-md

"""
import logging
import os
import re
import functools
import py_trees, py_trees_ros
from .actions import AddAllCollisionBoxesAct, MoveToStartAct, PickAct, PlaceAct, RemoveAllCollisionBoxAct

class Actor(object):
    def __init__(self):
        # Set up logging
        self._logger = logging.getLogger(__name__)

        # Get the directories for actor and solution file
        root_dir = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../../../..')
        self._problem_dir = os.path.abspath(root_dir + '/srp_md/pddl/problems')
        if not os.path.exists(self._problem_dir):
            raise IOError(self._problem_dir + ' does not exist')
        self._solution_file = os.path.abspath(self._problem_dir + '/input_gen_problem.pddl.soln')

    def act(self, solution_filename=None):
        self._logger.debug('Starting to act')

        # If solution filename is defined, change the solution filename
        if solution_filename is not None:
            self._logger.debug('Using given solution filename')
            self._solution_file = os.path.abspath(self._problem_dir + '/' + solution_filename)

        # Initialize the behavior tree
        self._logger.debug('Setting up behavior tree...')
        root = py_trees.composites.Sequence(name='srp_md_act')
        # py_trees.logging.level = py_trees.logging.Level.DEBUG
        # root.add_children([AddAllCollisionBoxesAct(name='srp_md'), MoveToStartAct(name='srp_md')])
        root.add_children([AddAllCollisionBoxesAct(name='srp_md')])
        # Read in the solution file, and do:
        with open(self._solution_file, "r") as solution:

            lines = list(solution)
            lines_copy = list(lines)
            print(type(solution))
            # For each line in solution, do:
            place_stack = []
            place_near = []
            relative_object_stack = []
            relative_object_near = []
            for i, line in enumerate(lines):
                line = re.sub('[()]+', '', line)
                action, obj_1, obj_2, _ = line.split()
                if "place" in action:
                    if "stack" in action:
                        place_stack.append(i)
                        relative_object_stack.append(obj_2)
                    if "near" in action:
                        place_near.append(i)
                        relative_object_near.append(i)

            print("place_stack: ", place_stack)

            # for i, line in enumerate(solution):
            for i, line in enumerate(lines_copy):
                print('ith is working')
                # Get rid of the parentheses
                line = re.sub('[()]+', '', line)

                # Get the words
                action, obj_1, obj_2, _ = line.split()

                # If the action includes word pick, do:
                if "pick" in action:
                    if i+1 in place_stack:
                    # Add pick action to the root
                        print('relation=Stacking')
                        index_relative_object = place_stack.index(i+1)
                        root.add_child(PickAct(i, obj_1, relative_object_stack[index_relative_object], relation='Stacking'))
                    elif i+1 in place_near:
                        index_relative_object = place_near.index(i+1)
                        root.add_child(PickAct(i, obj_1, relative_object_near[index_relative_object], relation='Near'))
                    else:
                        print('relation!=Stacking')
                        root.add_child(PickAct(i, obj_1, obj_2))
                        
                # If the action includes word place, do:
                if "place" in action:
                    # Add place action to the root
                    root.add_child(PlaceAct(i, obj_1, obj_2))

        # Build the behavior tree
        tree = py_trees_ros.trees.BehaviourTree(root)

        # Tick the tree
        self._logger.debug('Executing the behavior tree')
        tree.setup(timeout=10)

        while True:
            tree.tick()
            if tree.root.status == py_trees.Status.SUCCESS:
                tree.interrupt()
                tree.blackboard_exchange.unregister_services()
                self._logger.info('Succeded to execute the behavior tree!')
                return
            elif tree.root.status == py_trees.Status.FAILURE:
                tree.interrupt()
                tree.blackboard_exchange.unregister_services()
                self._logger.error('Action pipeline not successful!')
                


