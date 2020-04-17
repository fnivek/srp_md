""" Plan

Contains the planning utilities for srp-md

"""
import logging
import os
import subprocess


class Planner(object):
    def __init__(self):
        # Set up logging
        self._logger = logging.getLogger(__name__)

        # Get the directories for planner, domain, and problems
        root_dir = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../../../..')
        self._planner_dir = os.path.abspath(root_dir + '/submodules/pyperplan/src')
        self._domain_dir = os.path.abspath(root_dir + '/srp_md/pddl/domains')
        self._problem_dir = os.path.abspath(root_dir + '/srp_md/pddl/problems')
        for path in [self._planner_dir, self._domain_dir, self._problem_dir]:
            if not os.path.exists(path):
                raise IOError(path + ' does not exist')
        self._planner_executable = os.path.abspath(self._planner_dir + '/pyperplan.py')
        self._domain_file = os.path.abspath(self._domain_dir + '/domain.pddl')

    def plan(self, init_graph=None, goal_graph=None):
        self._logger.debug('Starting to plan')

        # If graphs are not inputs, just use default auto_gen_problem for debugging
        if init_graph is None or goal_graph is None:
            self._logger.debug('Using auto_gen_problem due to missing inputs')
            self._problem_file = os.path.abspath(self._problem_dir + '/auto_gen_problem.pddl')
        # Make the PDDL file from input graphs
        else:
            # Change the name of problem file here
            self._problem_file = os.path.abspath(self._problem_dir + '/input_gen_problem.pddl')

            # If the problem file already exists, delete the file
            if os.path.exists(self._problem_file):
                os.remove(self._problem_file)

            # Make the problem file with input graphs
            with open(self._problem_file, "w+") as f_prob:
                # Initialize indent and write basic structure
                ind = "  "
                num_ind = [0]
                write = lambda s: f_prob.write(ind * num_ind[0] + s)
                write("(define (problem current-scene-to-goal-scene)\n\n")
                num_ind[0] += 1
                write("(:domain srp-md)\n\n")

                # Write the objects
                write("(:objects\n")
                num_ind[0] += 1
                objects_str = ""
                for obj_name in init_graph.get_obj_names():
                    if obj_name == 'table':
                        continue
                    objects_str += obj_name + " "
                write(objects_str + "- object\n")
                write("table - surface\n")
                write("fetch_gripper - end_effector\n")
                num_ind[0] -= 1
                write(")\n\n")

                def graph_to_pddl(graph):
                    num_ind[0] += 1
                    sup_group = set([])
                    on_dict = {}
                    sup_dict = {}
                    self._logger.debug('Build on stacks')
                    for rel in graph.relations:
                        self._logger.debug(rel)
                        if rel.value == 'proximity':
                            self._logger.debug('proximity {} {}'.format(rel.obj1.name, rel.obj2.name))
                            write('(proximity {} {})\n'.format(rel.obj1.name, rel.obj2.name))
                        elif rel.value == 'on' or rel.value == 'support':
                            top_obj = rel.obj1.name
                            bot_obj = rel.obj2.name
                            if rel.value == "support":
                                self._logger.debug('support')
                                top_obj = rel.obj2.name
                                bot_obj = rel.obj1.name
                            self._logger.debug('top {} / bot {}'.format(top_obj, bot_obj))
                            sup_group.add(bot_obj)
                            try:
                                on_dict[top_obj].append(bot_obj)
                            except KeyError:
                                on_dict[top_obj] = [bot_obj]
                            try:
                                sup_dict[bot_obj].append(top_obj)
                            except KeyError:
                                sup_dict[bot_obj] = [top_obj]

                    self._logger.debug('sup_group %s', sup_group)
                    self._logger.debug('on_dict %s', on_dict)
                    self._logger.debug('sup_dict %s', sup_dict)
                    # Find the objects on top and bottom
                    self._logger.debug('Recurse over stacks')
                    above_set = set([])
                    for obj_name in graph.get_obj_names():
                        if obj_name not in sup_group:
                            self._logger.debug('{} is on top and clear'.format(obj_name))
                            write("(clear " + obj_name + ")\n")
                            # Recurse through top objects to corectly write add above attribute
                            def write_above(top_obj, above_objs=None):
                                self._logger.debug('top_obj %s', top_obj)
                                self._logger.debug('above_objs %s', above_objs)
                                above_objs = [] if above_objs is None else above_objs
                                try:
                                    bot_objs = on_dict[top_obj]
                                except KeyError:
                                    bot_objs = []
                                self._logger.debug('{} is on {}'.format(top_obj, bot_objs))
                                # Check if its only on the table
                                if len(bot_objs) == 1 and 'table' in bot_objs:
                                    self._logger.debug('{} is on the table'.format(top_obj))
                                    write("(at " + top_obj + " table)\n")
                                    return
                                for bot_obj in bot_objs:
                                    if bot_obj == 'table':
                                        self._logger.debug('{} is on the table'.format(top_obj))
                                        continue
                                    if not bool(set(bot_objs) & set(sup_dict[bot_obj])):
                                        self._logger.debug('{} is directly on {}'.format(top_obj, bot_obj))
                                        write("(on " + top_obj + " " + bot_obj + ")\n")
                                    above_str = "(above {} {})\n".format(top_obj, bot_obj)
                                    if above_str not in above_set:
                                        write(above_str)
                                        above_set.add(above_str)
                                    for above_obj in above_objs:
                                        above_str = "(above {} {})\n".format(above_obj, bot_obj)
                                        if above_str not in above_set:
                                            write(above_str)
                                            above_set.add(above_str)
                                    write_above(bot_obj, above_objs + [top_obj])
                            write_above(obj_name)

                    # Start with empty gripper hand
                    write("(free fetch_gripper)\n")
                    num_ind[0] -= 1
                    write(")\n\n")

                # Write the initial scene
                write("(:init\n")
                self._logger.debug('Converting {} to an init scene'.format(init_graph))
                graph_to_pddl(init_graph)

                # Write the goal scene
                write("(:goal\n")
                num_ind[0] += 1
                write("(and\n")
                self._logger.debug('Converting {} to a goal'.format(goal_graph))
                graph_to_pddl(goal_graph)
                num_ind[0] -= 1
                write(")\n")

                # End the paranthesis
                num_ind[0] -= 1
                write(")\n")

        # Plan from the generated PDDL file
        self._plan_cmd = [self._planner_executable, self._domain_file, self._problem_file]
        out = subprocess.Popen(self._plan_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()
        self._logger.debug(stdout)
        self._logger.debug(stderr)
        if stdout.find('Plan correct') != -1:
            self._logger.info('Plan correct')
        else:
            self._logger.warn('Failed to plan')
            return None

        self._soln_file = os.path.abspath(self._problem_file + '.soln')
        lines = []
        with open(self._soln_file, 'r') as soln:
            lines = soln.readlines()
        self._logger.debug(lines)
        return [line[1:-1].split() for line in lines]
