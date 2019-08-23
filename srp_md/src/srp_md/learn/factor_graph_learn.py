# Python imports
from __future__ import absolute_import
import logging
import itertools
import operator

# SRP MD imports
from . import learn
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._config_list = [(1, 0), (0, 1), (2, 0), (1, 1), (0, 2),
                             (3, 0), (2, 1), (1, 2), (0, 3)]

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
          A dictionary of LearnedFactor where key's are a tuple (num_objs, num_relations):
            {'xx': {x1=A,x2=B: 10, ...}, 'x': {x1=A: 10, x2=B: 0.01}, 'rrr': {r1=ON, r2=ON, r3=DISJOINT: 3.8}}

        """
        self._logger.debug('Learn')

        factor_gens = {}
        # Loop through all examples
        for graph in obs:
            # Loop through individual factors for one observation
            for factor in graph.gen_input_factors(configs=self._config_list):
                # Find the appropriate LearnedFactor to udpate
                num_relations = sum(1 for var in factor.vars if var.type is 'relation')
                num_objs = len(factor.vars) - num_relations
                gen_index = (num_objs, num_relations)
                gen = None
                if gen_index in factor_gens:
                    gen = factor_gens[gen_index]
                else:
                    gen = LearnedFactor(num_objs, num_relations)
                    factor_gens[gen_index] = gen

                # Update the learned factor
                gen.observe(tuple(sorted([var.value for var in factor.vars])))

        return factor_gens


class LearnedFactor():
    def __init__(self, num_objs, num_relations):
        self.num_objs = num_objs
        self.num_relations = num_relations
        self._freq = {}

    def observe(self, obs):
        # Increment seen assignments
        if obs in self._freq:
            self._freq[obs] += 1
        else:
            self._freq[obs] = 1

    def gen_factor(self, vars):
        # TODO(Kevin): Fill in the probabilites correctly
        return srp_md.Factor(vars, probs=[1 for _ in range(reduce(operator.mul, [var.num_states for var in vars]))])


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
