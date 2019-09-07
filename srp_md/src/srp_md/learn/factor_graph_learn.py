# Python imports
from __future__ import absolute_import
import logging
import operator
from collections import OrderedDict

# SRP MD imports
from . import learn
from .factor_learners import FreqFactorLearner
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._config_list = [(2, 1), (3, 3)]
        self.factor_learner = FreqFactorLearner

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
          A dictionary of FactorHandler where key's are a tuple (num_objs, num_relations):
            {'xx': {x1=A,x2=B: 10, ...}, 'x': {x1=A: 10, x2=B: 0.01}, 'rrr': {r1=ON, r2=ON, r3=DISJOINT: 3.8}}

        """
        self._logger.debug('Learn')

        factors = {}
        # Loop through all examples
        for graph in obs:
            # Loop through individual factors for one observation
            for factor in graph.gen_ordered_factors(configs=self._config_list):
                # Find the appropriate FactorHandler to udpate
                index = (len(factor.objs), len(factor.relations))
                if index not in factors:
                    self._logger.debug('Learning a new factor of type {}'.format(index))
                    factors[index] = FactorHandler(self.factor_learner())

                # Update the learned factor
                mb = graph.markov_blanket(factor.vars)
                # Actually order this because mb is a set which has no order
                factors[index].update_factor(self.generate_ord_dict(factor.vars), self.generate_ord_dict(mb))

        return factors

    def generate_ord_dict(self, var_list=[]):
        ord_dict = []
        for var in var_list:
            if isinstance(var, srp_md.Object):
                ord_dict.append((var, var.properties))
            elif isinstance(var, srp_md.Relation):
                ord_dict.append((var, {"value": var.value}))
            else:
                self._logger.error('Unidentified variable type detected!')
        return OrderedDict(ord_dict)


class FactorHandler():
    def __init__(self, learner=None):
        self._learner = learner
        if learner is None:
            self._learner = FreqFactorLearner()

    def update_factor(self, obs, markov_blanket):
        self._learner.observe(obs, markov_blanket)

    def generate_factor(self, vars):
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
        self._properties = OrderedDict([(var, None) for var in vars])
        # libDAI uses specific ordering of permutations, which reversing the list will match
        self._vars = list(reversed(vars))
        self._recurse_generate_factor()
        return srp_md.SgFactor(vars, self._probs)

    def _recurse_generate_factor(self, var_index=0):
        # Assign prob
        if var_index >= len(self._vars):
            self._probs[self._probs_index] = self._learner.predict(self._properties)
            self._probs_index += 1
            return

        var = self._vars[var_index]
        # Iterate over all relations
        if isinstance(var, srp_md.Relation):
            for relation in srp_md.SceneGraph.RELATION_STRS:
                self._properties[var] = {'value': relation}
                self._recurse_generate_factor(var_index + 1)
        else:
            # Only has one state
            self._properties[var] = var.properties
            self._recurse_generate_factor(var_index + 1)


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
