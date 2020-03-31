# Python imports
from __future__ import absolute_import
import logging
import operator
from collections import OrderedDict

# SRP MD imports
from . import learn
from .factor_learners import FreqFactorLearner, FACTOR_LEARNERS, SklearnFactorLearner, JointFreqFactorLearner
import srp_md


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        super(FactorGraphLearner, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._factors_to_learn = [2]
        self.factor_learner = 'decision_tree'
        self._allowed_config_keys.extend(['factor_learner', 'factors_to_learn'])

    @property
    def factor_learner(self):
        for key, value in FACTOR_LEARNERS.iteritems():
            if value == self._factor_learner:
                return key
        return None

    @factor_learner.setter
    def factor_learner(self, factor_learner):
        self._factor_learner = FACTOR_LEARNERS[factor_learner]

    @property
    def factors_to_learn(self):
        return self._factors_to_learn

    @factors_to_learn.setter
    def factors_to_learn(self, factors):
        """ Should be an iterable with each element an int representing the number of objects per factor. """
        for num_objs in factors:
            if num_objs > 4:
                self._logger.warn('{0} object factors must evaluate 6^(ncr({0}, 2)) = {1} probabilities'.format(
                    num_objs, pow(6, srp_md.ncr(num_objs, 2))))
        self._factors_to_learn = factors

    def prop_to_cats(self, properties, num_objs):
        return [properties[key] for key in properties.keys()] * num_objs

    def learn(self, obs, properties):
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
            # configs = [(num_objs, srp_md.ncr(num_objs, 2)) for num_objs in self._factors_to_learn]
            for factor in graph.gen_ordered_factor():
                # Find the appropriate FactorHandler to udpate
                index = (len(factor.objs), len(factor.relations))
                if index not in factors:
                    self._logger.debug('Learning a new factor of type {}'.format(index))
                    if issubclass(self._factor_learner, SklearnFactorLearner):
                        category = self.prop_to_cats(properties, len(factor.objs))
                        factors[index] = FactorHandler(self._factor_learner(category=category))
                    else:
                        factors[index] = FactorHandler(self._factor_learner())

                # Update the learned factor
                mb = graph.markov_blanket(factor.vars)
                # Actually order this because mb is a set which has no order
                factors[index].update_factor(self.generate_ord_dict(factor.vars), self.generate_ord_dict(mb))

            # Generate the relation counting factors
            for obj in graph.objs:
                for rel_value in ['on', 'support', 'in', 'contain', 'proximity']:
                    # Count all relations with obj as the first object
                    count = sum(1 for rel in graph.relations if ((rel.obj1 is obj) and (rel.value == rel_value)) or
                        ((rel.obj2 is obj) and (rel.value == srp_md.Relation.REV_RELATION_DICT[rel_value])))
                    key = (obj.assignment['class'], rel_value)
                    try:
                        factors[key].update_factor({key: {'count': count}})
                    except KeyError:
                        factors[key] = CardinalityFactorHandler(obj, *key)
                        factors[key].update_factor({key: {'count': count}})

        return factors

    def generate_ord_dict(self, vars):
        return OrderedDict((var, var.assignment) for var in vars)


class FactorHandler(object):
    def __init__(self, learner=None):
        self.factor_type = 'object_relation'
        self._learner = learner
        if learner is None:
            self._learner = FreqFactorLearner()

    def update_factor(self, obs, markov_blanket=None):
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
        self._assignment = OrderedDict([(var, None) for var in vars])
        # libDAI uses specific ordering of permutations, which reversing the list will match
        self._vars = list(reversed(vars))
        self._recurse_generate_factor()
        return srp_md.SgFactor(vars, self._probs)

    def _recurse_generate_factor(self, var_index=0):
        # Assign prob
        if var_index >= len(self._vars):
            self._probs[self._probs_index] = self._learner.predict(self._assignment) + 1e-9  # Add uniform bias to the score output
            self._probs_index += 1
            return

        var = self._vars[var_index]
        # Iterate over all relations
        if isinstance(var, srp_md.Relation):
            for relation in srp_md.Relation.RELATION_STRS:
                self._assignment[var] = {'value': relation}
                self._recurse_generate_factor(var_index + 1)
        else:
            # Only has one state
            self._assignment[var] = var.assignment
            self._recurse_generate_factor(var_index + 1)


class CardinalityFactorHandler(FactorHandler):
    def __init__(self, obj, class_name, rel_value):
        super(CardinalityFactorHandler, self).__init__(learner=JointFreqFactorLearner())
        self.factor_type = 'cardinality'
        self._obj = obj
        self._class_name = class_name
        self._rel_value = rel_value

    def _recurse_generate_factor(self, var_index=0, cached_predictions=None):
        if cached_predictions is None:
            cached_predictions = {}
        # Assign prob
        if var_index >= len(self._vars):
            count = sum(1 for assignment in self._assignment.values() if assignment['value'] == self._rel_value)
            try:
                self._probs[self._probs_index] = cached_predictions[count]
            except KeyError:
                score = self._learner.predict({None: {'count': count}}) + 1e-3  # Add uniform bias to the score output
                self._probs[self._probs_index] = score
                cached_predictions[count] = score

            self._probs_index += 1
            return

        var = self._vars[var_index]
        # Iterate over all relations
        for relation in srp_md.Relation.RELATION_STRS:
            # Flip the relationship if the object is the second part of the relation
            if var.obj2 == self._obj:
                relation = srp_md.Relation.REV_RELATION_DICT[relation]
            self._assignment[var] = {'value': relation}
            self._recurse_generate_factor(var_index + 1, cached_predictions)


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
