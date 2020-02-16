""" Act

Contains the robot acting utilities for srp-md

"""
import logging
import os
import re
import functools
import py_trees, py_trees_ros
from .actions import AddAllCollisionBoxesAct, MoveToStartAct, PickAct, PlaceAct

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
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        root.add_children([AddAllCollisionBoxesAct(name='srp_md'), MoveToStartAct(name='srp_md')])

        # Read in the solution file, and do:
        with open(self._solution_file, "r") as solution:

            # For each line in solution, do:
            for i, line in enumerate(solution):

                # Get rid of the parentheses
                line = re.sub('[()]+', '', line)

                # Get the words
                action, obj_1, obj_2, _ = line.split()

                # If the action includes word pick, do:
                if "pick" in action:

                    # Add pick action to the root
                    root.add_child(PickAct(i, obj_1))

                # If the action includes word place, do:
                if "place" in action:

                    # Add place action to the root
                    root.add_child(PlaceAct(i, obj_1, obj_2))

        # Build the behavior tree
        tree = py_trees_ros.trees.BehaviourTree(root)

        # Tick the tree
        self._logger.debug('Executing the behavior tree')
        tree.setup(timeout=10)

        if not py_trees.Status.SUCCESS or not py_trees.Status.FAILURE:
            tree.tick()
        else if py_trees.Status.SUCCESS:
            return
        else:
            self._logger.error('Action pipeline not successful!')
            sys.exit(1)




