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
                ind = ""
                f_prob.write(ind + "(define (problem current-scene-to-goal-scene)\n\n")
                ind += "\t"
                f_prob.write(ind + "(:domain srp-md)\n\n")

                # Write the objects
                f_prob.write(ind + "(:objects\n")
                ind += "\t"
                objects_str = ""
                for obj_name in init_graph.get_obj_names():
                    objects_str += obj_name + " "
                f_prob.write(ind + objects_str + "- object\n")
                f_prob.write(ind + "table - surface\n")
                f_prob.write(ind + "fetch_gripper - end_effector\n")
                ind = ind[:-1]
                f_prob.write(ind + ")\n\n")

                # Write the initial scene
                f_prob.write(ind + "(:init\n")
                ind += "\t"
                on_group = set([])
                sup_group = set([])
                for rel in init_graph.relations:
                    if rel.value == "on":
                        f_prob.write(ind + "(on " + rel.obj1.name + " " + rel.obj2.name + ")\n")
                        on_group.add(rel.obj1.name)
                        sup_group.add(rel.obj2.name)
                    elif rel.value == "support":
                        f_prob.write(ind + "(on " + rel.obj2.name + " " + rel.obj1.name + ")\n")
                        on_group.add(rel.obj2.name)
                        sup_group.add(rel.obj1.name)
                for obj_name in init_graph.get_obj_names():
                    if obj_name not in on_group:
                        f_prob.write(ind + "(at " + obj_name + " table)\n")
                    if obj_name not in sup_group:
                        f_prob.write(ind + "(clear " + obj_name + ")\n")
                f_prob.write(ind + "(free fetch_gripper)\n")
                ind = ind[:-1]
                f_prob.write(ind + ")\n\n")

                # Write the goal scene
                f_prob.write(ind + "(:goal\n")
                ind += "\t"
                f_prob.write(ind + "(and\n")
                ind += "\t"
                on_group = set([])
                sup_group = set([])
                for rel in goal_graph.relations:
                    if rel.value == "on":
                        f_prob.write(ind + "(on " + rel.obj1.name + " " + rel.obj2.name + ")\n")
                        on_group.add(rel.obj1.name)
                        sup_group.add(rel.obj2.name)
                    elif rel.value == "support":
                        f_prob.write(ind + "(on " + rel.obj2.name + " " + rel.obj1.name + ")\n")
                        on_group.add(rel.obj2.name)
                        sup_group.add(rel.obj1.name)
                for obj_name in goal_graph.get_obj_names():
                    # if obj_name not in on_group:
                    #     f_prob.write(ind + "(at " + obj_name + " table)\n")
                    if obj_name not in sup_group:
                        f_prob.write(ind + "(clear " + obj_name + ")\n")
                f_prob.write(ind + "(free fetch_gripper)\n")
                ind = ind[:-1]
                f_prob.write(ind + ")\n")
                ind = ind[:-1]
                f_prob.write(ind + ")\n")

                # End the paranthesis
                ind = ind[:-1]
                f_prob.write(ind + ")\n")



        # Plan from the generated PDDL file
        self._plan_cmd = [self._planner_executable, self._domain_file, self._problem_file]
        out = subprocess.Popen(self._plan_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()
        print stdout, stderr

        self._soln_file = os.path.abspath(self._problem_file + '.soln')
        with open(self._soln_file, 'r') as soln:
            print soln.readlines()
