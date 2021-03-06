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
        self._planner_dir = os.path.abspath('/opt/downward/')
        self._domain_dir = os.path.abspath(root_dir + '/srp_md/pddl/domains')
        self._problem_dir = os.path.abspath(root_dir + '/srp_md/pddl/problems')
        for path in [self._planner_dir, self._domain_dir, self._problem_dir]:
            if not os.path.exists(path):
                raise IOError(path + ' does not exist')
        self._planner_executable = os.path.abspath(self._planner_dir + '/fast-downward.py')
        self._domain_file = os.path.abspath(self._domain_dir + '/domain.pddl')

        # Define options
        self._search_options = {
            # Optimal
            'dijkstra': ['--evaluator', 'h=blind(transform=adapt_costs(cost_type=NORMAL))',
                         '--search', 'astar(h,cost_type=NORMAL,max_time=30)'],
            'max-astar': ['--evaluator', 'h=hmax(transform=adapt_costs(cost_type=NORMAL))',
                          '--search', 'astar(h,cost_type=NORMAL,max_time=%s,bound=%s)'],
            'cerberus': ['--evaluator', 'h=hmax(transform=adapt_costs(cost_type=NORMAL))',
                         '--search', 'astar(h,cost_type=NORMAL,max_time=%s,bound=%s)'],
            'lmcut-astar': ['--evaluator', 'h=lmcut(transform=adapt_costs(cost_type=NORMAL))',
                            '--search', 'astar(h,cost_type=NORMAL,max_time=%s,bound=%s)'],

            # Suboptimal
            'ff-astar': ['--evaluator', 'h=ff(transform=adapt_costs(cost_type=NORMAL))',
                         '--search', 'astar(h,cost_type=NORMAL,max_time=90)'],
            'ff-eager': ['--evaluator', 'h=ff(transform=adapt_costs(cost_type=PLUSONE))',
                         '--search', 'eager_greedy([h],max_time=%s,bound=%s)'],
            'ff-eager-pref': ['--evaluator', 'h=ff(transform=adapt_costs(cost_type=PLUSONE))',
                              '--search', 'eager_greedy([h],preferred=[h],max_time=%s,bound=%s)'],
            'ff-lazy': ['--evaluator', 'h=ff(transform=adapt_costs(cost_type=PLUSONE))',
                        '--search', 'lazy_greedy([h],preferred=[h],max_time=%s,bound=%s)'],
            'goal-lazy': ['--evaluator', 'h=goalcount(transform=no_transform())',
                          '--search', 'lazy_greedy([h],randomize_successors=True,max_time=30,bound=40)'],
            'add-random-lazy': ['--evaluator', 'h=add(transform=adapt_costs(cost_type=PLUSONE))',
                                '--search', 'lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)'],

            'ff-eager-tiebreak': ['--evaluator', 'h=ff(transform=no_transform())',
                                  '--search', 'eager(tiebreaking([h, g()]),reopen_closed=false,'
                                  'cost_type=NORMAL,max_time=%s,bound=%s, f_eval=sum([g(), h]))'],  # preferred=[h],
            'ff-lazy-tiebreak': ['--evaluator', 'h=ff(transform=no_transform())',
                                 '--search', 'lazy(tiebreaking([h, g()]),reopen_closed=false,'
                                 'randomize_successors=True,cost_type=NORMAL,max_time=%s,bound=%s)'],  # preferred=[h],

            'ff-ehc': ['--evaluator', 'h=ff(transform=adapt_costs(cost_type=NORMAL))'
                       '--search', 'ehc(h,preferred=[h],preferred_usage=RANK_PREFERRED_FIRST,'
                       'cost_type=NORMAL,max_time=%s,bound=%s)'],
            'astar': ['--search', 'astar(blind())'],
            # The key difference is that ehc resets the open list upon finding an improvement

        }
        for w in range(1, 1+5):
            self._search_options['ff-wastar{}'.format(w)] = [
                '--evaluator', 'h=ff(transform=adapt_costs(cost_type=NORMAL))',
                '--search', 'lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w={},'
                'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)'.format(w)]
            self._search_options['cea-wastar{}'.format(w)] = [
                '--evaluator', 'h=cea(transform=adapt_costs(cost_type=PLUSONE))',
                '--search', 'lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w={},'
                'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)'.format(w)]

        self._default_planner = 'ff-astar'

    def plan(self, init_graph=None, goal_graph=None, problem_file=None, soln_file=None):
        self._logger.debug('Starting to plan')

        # If graphs are not inputs, just use default auto_gen_problem for debugging
        if init_graph is None or goal_graph is None:
            self._logger.error('Missing initial scene graph or goal scene graph')
            return None
        # Make the PDDL file from input graphs
        else:
            # Change the name of problem file here
            self._problem_file = problem_file
            if self._problem_file is None:
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
                            continue
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
                    directly_on_dict = {}
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
                                    directly_on_dict[top_obj] = 'table'
                                    return
                                for bot_obj in bot_objs:
                                    if bot_obj == 'table':
                                        self._logger.debug('{} is on the table'.format(top_obj))
                                        continue
                                    if not bool(set(bot_objs) & set(sup_dict[bot_obj])):
                                        self._logger.debug('{} is directly on {}'.format(top_obj, bot_obj))
                                        write("(on " + top_obj + " " + bot_obj + ")\n")
                                        directly_on_dict[top_obj] = bot_obj
                                    above_str = "(above {} {})\n".format(top_obj, bot_obj)
                                    if above_str not in above_set:
                                        # write(above_str)
                                        above_set.add(above_str)
                                    for above_obj in above_objs:
                                        above_str = "(above {} {})\n".format(above_obj, bot_obj)
                                        if above_str not in above_set:
                                            # write(above_str)
                                            above_set.add(above_str)
                                    write_above(bot_obj, above_objs + [top_obj])
                            write_above(obj_name)
                    # Handle proximity
                    for rel in graph.relations:
                        if (rel.value == 'proximity'):
                            try:
                                if directly_on_dict[rel.obj1.name] == directly_on_dict[rel.obj2.name]:
                                    self._logger.debug('proximity {} {}'.format(rel.obj1.name, rel.obj2.name))
                                    write('(proximity {} {})\n'.format(rel.obj1.name, rel.obj2.name))
                            except KeyError:
                                pass

                    # Start with empty gripper hand
                    write("(free fetch_gripper)\n")

                # Write the initial scene
                write("(:init\n")
                self._logger.debug('Converting {} to an init scene'.format(init_graph))
                graph_to_pddl(init_graph)
                num_ind[0] -= 1
                write(")\n\n")

                # Write the goal scene
                write("(:goal\n")
                num_ind[0] += 1
                write("(and\n")
                self._logger.debug('Converting {} to a goal'.format(goal_graph))
                graph_to_pddl(goal_graph)
                for obj in goal_graph.get_obj_names():
                    if obj == 'table':
                        continue
                    write("(in_box {})\n".format(obj))
                num_ind[0] -= 1
                write(")\n\n")
                num_ind[0] -= 1
                write(")\n")

                # End the paranthesis
                num_ind[0] -= 1
                write(")\n")

        # Plan from the generated PDDL file
        self._soln_file = soln_file
        if self._soln_file is None:
            self._soln_file = os.path.abspath(self._problem_file + '.soln')
        self._plan_cmd = [
            self._planner_executable,
            # '--validate',
            '--plan-file',
            self._soln_file,
            self._domain_file,
            self._problem_file
        ] + self._search_options[self._default_planner]
        # ] + self._search_options['goal-lazy']
        self._logger.debug('Plan command: {}'.format(self._plan_cmd))
        out = subprocess.Popen(self._plan_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()
        self._logger.debug(stdout)
        self._logger.debug(stderr)
        if stdout.find('search exit code: 0') != -1:
            self._logger.info('Plan correct')
        else:
            self._logger.info('Failed to plan')
            return None

        lines = []
        with open(self._soln_file, 'r') as soln:
            lines = soln.readlines()
        self._logger.debug(lines)
        return [line[1:-2].split() for line in lines[:-1]]
