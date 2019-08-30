# Python imports
from __future__ import absolute_import
import logging
import itertools
import operator
from random import random

# SRP MD imports
from . import learn
from .decision_tree_factor_learner import DecisionTreeFactorLearner
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        # self._config_list = [(0, 1), (1, 1), (0, 2), (2, 1), (1, 2), (0, 3)]
        self._config_list = [(2, 1)]
        self.factor_learner = DecisionTreeFactorLearner

    def set_attributes(self, **kwargs):
        for key, value in kwargs.iteritems():
            self._logger.debug('Setting {} to {}'.format(key, value))
            setattr(self, key, value)

    def learn(self, obs):
        """ Learn.

        Transforms a set of scene graphs into learned factors that can be used to build scene graphs. Each factor in the
        input scene graph can be maped to a particular type of factor e.g., a object unary potential. Duplicates in the
        graph may exist and these are treated as the same factor and learned as one. Additionally there may be duals,
        where a dual is a pair of factors that are related and one can be derived from the other. An example of a  dual
        is a factor between a relation and an object1 and a factor between relation and object 2 because if object 1 and
        2 are swaped, the relation just flips to its dual value (ON <--> SUPPORT). Duals are treated as one factor and
        learned together with a dual function to generate the dual factor.

        Input:
          obs - list of srp_md.SceneGraph

        Returns:
          A dictionary of FactorGenerator where key's are a tuple (num_objs, num_relations):
            {'xx': {x1=A,x2=B: 10, ...}, 'x': {x1=A: 10, x2=B: 0.01}, 'rrr': {r1=ON, r2=ON, r3=DISJOINT: 3.8}}

        """
        self._logger.debug('Learn')

        factor_gens = {}
        # Loop through all examples
        for graph in obs:
            # Loop through individual factors for one observation
            for factor in graph.gen_input_factors(configs=self._config_list):
                # Find the appropriate FactorGenerator to udpate
                num_relations = sum(1 for var in factor.vars if var.type is 'relation')
                num_objs = len(factor.vars) - num_relations
                gen_index = (num_objs, num_relations)
                gen = None
                if gen_index in factor_gens:
                    gen = factor_gens[gen_index]
                else:
                    gen = FactorGenerator(num_objs, num_relations, self.factor_learner())
                    factor_gens[gen_index] = gen

                # Update the learned factor
                gen.observe(tuple(sorted([var.properties['value'] for var in factor.vars])))

        return factor_gens


class FactorGenerator():
    def __init__(self, num_objs, num_relations, learner=None):
        self.num_objs = num_objs
        self.num_relations = num_relations
        self._learner = learner
        if learner is None:
            # Default to FreqFactorLearner
            self._learner = FreqFactorLearner()
        setattr(FactorGenerator, 'observe', self._learner.observe)

    def gen_factor(self, vars):
        """
        Generates factors from vars.

        Uses the learned knowledge to generate a factor that cooresponds to the variables in vars. vars might contain
        variables that were not seen in the training data.

        WARNING: Vars must be ordered such that all objects are first and are followed by any relations
                 Then follow the rules specified in the Factor class defined in factor_graph.py

        """
        # Recursively determine every possible value for Factor.probs
        self._probs = [0 for _ in range(reduce(operator.mul, [var.num_states for var in vars]))]
        self._probs_index = 0
        self._assignment = {var: 0 for var in vars}
        self._vars = list(reversed(vars))
        self._recurse_gen_factor(0)
        return srp_md.Factor(vars, self._probs)

    def _recurse_gen_factor(self, var_index):
        # Assign prob
        if var_index >= len(self._vars):
            self._probs[self._probs_index] = self._learner.predict(self._assignment)
            self._probs_index += 1
            return

        var = self._vars[var_index]
        # Itterate over all relations
        if var.type == 'relation':
            for relation in srp_md.SceneGraph.RELATION_STRS:
                self._assignment[var] = relation
                self._recurse_gen_factor(var_index + 1)
        else:
            # Only has one state
            self._assignment[var] = var.properties['value']
            self._recurse_gen_factor(var_index + 1)


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
