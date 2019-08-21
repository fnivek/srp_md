# Python imports
from __future__ import absolute_import
import logging
import itertools

# SRP MD imports
from . import learn
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

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
          A dictionary of factor types to dictionaries of variable state to real [0, inf) i.e.:
            {'xx': {x1=A,x2=B: 10, ...}, 'x': {x1=A: 10, x2=B: 0.01}, 'rrr': {r1=ON, r2=ON, r3=DISJOINT: 3.8}}

        """
        self._logger.debug('Learn')

        factor_gen = {}
        for example in obs:
            obs_factor_gens = self.learn_some_factors(example)
            for factor_key in obs_factor_gens:
                if factor_key in factor_gen:
                    for value_key in obs_factor_gens[factor_key]:
                        if value_key in factor_gen[factor_key]:
                            factor_gen[factor_key][value_key] += obs_factor_gens[factor_key][value_key]
                        else:
                            factor_gen[factor_key][value_key] = obs_factor_gens[factor_key][value_key]
                else:
                    factor_gen[factor_key] = obs_factor_gens[factor_key]

        return factor_gen

    def learn_all_factors(self, graph):
        # Determine which factors exist in observation by counting number of objects
        # TODO(Kevin): When objects or relations has duplicate objects they are counted as different objects. I need to
        #              think about the assignment of the variable vs the number of variables
        factor_gens = {}
        for factor in graph.gen_all_possible_factors():
            # Produce the map index for the factor generator
            var_names = tuple([var.name for var in factor.vars])
            factor_index = tuple(sorted([var.value for var in factor.vars]))
            # Find the appropriate factor to udpate
            num_relations = 0
            for i in [1 for name in var_names if name.startswith('R_')]:
                num_relations += 1
            num_objs = len(factor_index) - num_relations
            gen_index = (num_objs, num_relations)
            gen = {}
            if gen_index in factor_gens:
                gen = factor_gens[gen_index]
            else:
                factor_gens[gen_index] = gen

            # Increment seen assignments
            if factor_index in gen:
                gen[factor_index] += 1
            else:
                gen[factor_index] = 1

        return factor_gens

    def learn_some_factors(self, graph):
        factor_gens = {}
        config_list = [(1, 0), (0, 1), (2, 0), (1, 1), (0, 2),
                       (3, 0), (2, 1), (1, 2), (0, 3)]
        for factor in graph.gen_input_factors(configs=config_list):
            # Produce the map index for the factor generator
            var_names = tuple([var.name for var in factor.vars])
            factor_index = tuple(sorted([var.value for var in factor.vars]))
            # Find the appropriate factor to udpate
            num_relations = 0
            for i in [1 for name in var_names if name.startswith('R_')]:
                num_relations += 1
            num_objs = len(factor_index) - num_relations
            gen_index = (num_objs, num_relations)
            gen = {}
            if gen_index in factor_gens:
                gen = factor_gens[gen_index]
            else:
                factor_gens[gen_index] = gen

            # Increment seen assignments
            if factor_index in gen:
                gen[factor_index] += 1
            else:
                gen[factor_index] = 1

        return factor_gens


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
