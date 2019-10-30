""" Plan

Contains the planning utilities for srp-md

"""
import logging
import os
import subprocess


class Planner(object):
    def __init__(self):
        # Logging
        self._logger = logging.getLogger(__name__)

        # Files and paths
        this_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.abspath(this_dir + '/../../../..')
        self._planner_dir = os.path.abspath(root_dir + '/submodules/pyperplan/src')
        self._domain_dir = os.path.abspath(root_dir + '/srp_md/pddl/domains')
        self._problem_dir = os.path.abspath(root_dir + '/srp_md/pddl/problems')
        for path in [self._planner_dir, self._domain_dir, self._problem_dir]:
            if not os.path.exists(path):
                raise IOError(path + ' does not exist')
        self._planner_executable = os.path.abspath(self._planner_dir + '/pyperplan.py')
        self._domain_file = os.path.abspath(self._domain_dir + '/domain.pddl')
        self._problem_file = os.path.abspath(self._problem_dir + '/auto_gen_problem.pddl')
        self._soln_file = os.path.abspath(self._problem_dir + '/auto_gen_problem.pddl.soln')
        self._plan_cmd = [self._planner_executable, self._domain_file, self._problem_file]

    def plan(self):
        self._logger.debug('Starting to plan')
        out = subprocess.Popen(self._plan_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()
        print stdout, stderr
        with open(self._soln_file, 'r') as soln:
            print soln.readlines()
