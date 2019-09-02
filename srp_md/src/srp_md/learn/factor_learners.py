# Python
import string
from random import random
import operator
from functools import reduce

# Scikit
from sklearn import linear_model, tree, preprocessing
from sklearn.pipeline import Pipeline

# Numpy
import numpy

# Project
import srp_md


# A dictionary of names to Factor Learner Classes
FACTOR_LEARNERS = {}


class JointFreqFactorLearner:
    """ Learn factors by frequency.

    Simpily keep a list of observation and return the count.

    """
    def __init__(self):
        # List of dictionaries of dictionaries [{{}}]
        self._joint_assignments = {}

    def to_joint(self, assignment):
        return tuple(reduce(operator.add, [var_assignment.values() for var_assignment in assignment.values()]))

    def observe(self, obs, markov_blanket):
        # Add joint assignment
        joint_assignment = self.to_joint(obs)
        if joint_assignment in self._joint_assignments:
            self._joint_assignments[joint_assignment] += 1
        else:
            self._joint_assignments[joint_assignment] = 1

    def predict(self, assignment):
        key = self.to_joint(assignment)
        if key in self._joint_assignments:
            return self._joint_assignments[key]

        return 0


FACTOR_LEARNERS['joint_frequency'] = JointFreqFactorLearner


class FreqFactorLearner:
    """ Learn factors by frequency.

    On observation keep a list of how many times each value of each property of each variable is seen.

    On prediction add up all the number of times the property assignments have been seen.

    """
    def __init__(self):
        # List of dictionaries of dictionaries [{{}}]
        self._assignments = []

    def observe(self, obs, markov_blanket):
        for var_index, value in enumerate(obs.values()):
            # Grow the assignments if needed
            if var_index >= len(self._assignments):
                self._assignments.append({})
            srp_md.count_dicts(value, self._assignments[var_index])

    def predict(self, assignment):
        count = 0
        for var_index, var_assignment in enumerate(assignment.values()):
            for prop, value in var_assignment.iteritems():
                try:
                    count += self._assignments[var_index][prop][value]
                except KeyError:
                    pass

        return count


FACTOR_LEARNERS['frequency'] = FreqFactorLearner


class DecisionTreeFactorLearner:
    """ Learn factors by a decision  tree.

    TODO(Kevin): Handle factors with more than one relation var
    TODO(Kevin): Handle real object domains
    TODO(Kevin): Tune the paramaters of the decision tree
    TODO(Kevin): Add a feature removal stage to the pipeline

    """
    def __init__(self):
        self._data = []
        self._target = []
        self._must_fit = True
        self._clf = tree.DecisionTreeClassifier()
        letters = [c for c in string.ascii_lowercase] + [c for c in string.ascii_uppercase]
        self._enc = preprocessing.OneHotEncoder(categories=[letters, letters])
        self._pipe = Pipeline([('enc', self._enc), ('tree', self._clf)])

    def observe(self, obs, markov_blanket):
        self._data.append([obj for obj in obs if obj not in srp_md.SceneGraph.RELATION_STRS])
        self._target.append([rel for rel in obs if rel in srp_md.SceneGraph.RELATION_STRS])
        self._must_fit = True

    def predict(self, assignment):
        # Fit data if needed
        if self._must_fit:
            self._must_fit = False
            self._pipe.fit(self._data, self._target)
        # Predict probability
        # TODO(Kevin): I don't think these should be sorted anymore
        value_tuple = tuple(sorted(value for value in assignment.values()
                            if value not in srp_md.SceneGraph.RELATION_STRS))
        probs = self._pipe.predict_proba([value_tuple])

        relation = [value for value in assignment.values() if value in srp_md.SceneGraph.RELATION_STRS][0]
        if relation in self._clf.classes_:
            probs_index = numpy.where(self._clf.classes_ == relation)
            return probs[0][probs_index[0][0]]

        return 0


# TODO(Kevin): Update to work with OrderedDictionary inputs
# FACTOR_LEARNERS['decision_tree'] = DecisionTreeFactorLearner


class LeastSquaresFactorLearner:
    """ Learn factors by a decision  tree.

    TODO(Kevin): Handle factors with more than one relation var
    TODO(Kevin): Handle real object domains
    TODO(Kevin): Tune the paramaters of the decision tree
    TODO(Kevin): Add a feature removal stage to the pipeline

    """
    def __init__(self):
        self._data = []
        self._target = []
        self._must_fit = True
        self._clf = linear_model.LinearRegression()
        letters = [c for c in string.ascii_lowercase] + [c for c in string.ascii_uppercase]
        self._enc = preprocessing.OneHotEncoder(categories=[letters, letters, srp_md.SceneGraph.RELATION_STRS])
        self._pipe = Pipeline([('enc', self._enc), ('least_squares', self._clf)])

    def observe(self, obs, markov_blanket):
        self._data.append(obs)
        self._target.append(1)
        self._must_fit = True

    def predict(self, assignment):
        # Fit data if needed
        if self._must_fit:
            self._must_fit = False
            self._pipe.fit(self._data, self._target)
        # Predict probability
        # TODO(Kevin): I don't think these should be sorted anymore
        value_tuple = tuple(sorted(value for value in assignment.values()))
        probs = self._pipe.predict([value_tuple])

        # relation = [value for value in assignment.values() if value in srp_md.SceneGraph.RELATION_STRS][0]
        # if relation in self._clf.classes_:
        #     probs_index = numpy.where(self._clf.classes_ == relation)
        #     return probs[0][probs_index[0][0]]

        return 0


# TODO(Kevin): Update to work with OrderedDictionary inputs
# FACTOR_LEARNERS['least_squares'] = LeastSquaresFactorLearner


class ClosedFormFactorLearner:
    def __init__(self):
        pass

    def observe(self, obs):
        pass

    def predict(self, assignment):
        return 0


FACTOR_LEARNERS['closed_form'] = ClosedFormFactorLearner
