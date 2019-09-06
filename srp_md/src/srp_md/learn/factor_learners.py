# Python
import string
from random import random
import operator
from functools import reduce
from collections import OrderedDict, Counter
from itertools import chain
from math import log, exp

# Scikit
from sklearn import linear_model, tree, preprocessing, svm
from sklearn.pipeline import Pipeline

# Numpy
import numpy

# Project
import srp_md


# Functions for working with assignments which are:
#   OrderedDict((srp_md.var, {'property_name', 'vaule', ...}), (...), ...)
def assign_to_vals(a):
    """ Convert an assignment to an ordered list of values. """
    return list(reduce(operator.add, [assign.values() for assign in a.values()], []))


def assign_eq(a, b):
    """ Check for equality between two assignments. """
    return assign_to_vals(a) == assign_to_vals(b)


# def assign_obj_var_getter(a):
#     """ Returns a function that will grab all indicies corresponding to object vars. """
#     return operator.itemgetter(*[i for i, var in enumerate(a.keys()) if var.type == 'object'])


# def assign_rel_var_getter(a):
#     """ Returns a function that will grab all indicies corresponding to relation vars. """
#     return operator.itemgetter(*[i for i, var in enumerate(a.keys()) if var.type == 'relation'])


def assign_obj_vals(a):
    """ Returns object vals. """
    return list(reduce(operator.add, [assign.values() for var, assign in a.iteritems() if var.type == 'object']))


def assign_rel_vals(a):
    """ Returns object vals. """
    return list(reduce(operator.add, [assign.values() for var, assign in a.iteritems() if var.type == 'relation']))


def assign_obj_val_indices(a):
    """ Returns all value indices of object vars. """
    indicies = []
    cur_index = 0
    for var, assignment in a.iteritems():
        num_props = len(assignment)
        if var.type != 'object':
            cur_index += num_props
            continue
        indicies += list(range(cur_index, cur_index + num_props))
        cur_index += num_props
    return indicies


# A dictionary of names to Factor Learner Classes
FACTOR_LEARNERS = {}


class JointFreqFactorLearner:
    """ Learn factors by frequency.

    Simpily keep a list of observation and return the count.

    """
    def __init__(self):
        # List of dictionaries of dictionaries [{{}}]
        self._joint_assignments = {}

    def observe(self, obs, markov_blanket):
        # Add joint assignment
        joint_assignment = tuple(assign_to_vals(obs))
        if joint_assignment in self._joint_assignments:
            self._joint_assignments[joint_assignment] += 1
        else:
            self._joint_assignments[joint_assignment] = 1

    def predict(self, assignment):
        key = tuple(assign_to_vals(assignment))
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


class SklearnFactorLearner(object):
    """ Learn factors by a decision  tree.

    TODO(Kevin): Get all possible categories so that we can use small numbers of examples and new novel objects
    TODO(Kevin): Tune the paramaters of the decision tree
    TODO(Kevin): Add a feature removal stage to the pipeline

    """
    def __init__(self, clf):
        self._data = []
        self._target = []
        self._must_fit = True
        self._clf = clf
        self._enc = preprocessing.OneHotEncoder()
        self._pipe = Pipeline([('enc', self._enc), ('tree', self._clf)])
        self._last_vals = None
        self._last_probs = None

    def _combine_rels(self, rels):
        """ Combine relations into one string deliminated by spaces. """
        return reduce(lambda x, y: x + ' ' + y, rels)

    def observe(self, obs, markov_blanket):
        self._data.append(assign_obj_vals(obs))
        # Each possible combination of assignments of the relations is a class
        self._target.append(self._combine_rels(assign_rel_vals(obs)))
        # New data so will need to refit
        self._must_fit = True

    def predict(self, assignment):
        # Fit data if needed
        if self._must_fit:
            self._must_fit = False
            self._last_vals = None
            self._pipe.fit(self._data, self._target)
        # Predict probability
        cur_vals = assign_obj_vals(assignment)
        if cur_vals != self._last_vals:
            self._probs = self._pipe.predict_proba([cur_vals])
            self._last_vals = cur_vals
        # Get the probability corresponding to the current assignment
        rel_vals = self._combine_rels(assign_rel_vals(assignment))
        prob = self._probs[0][self._clf.classes_ == rel_vals]
        # If the class has never been seen before return 0
        if len(prob) != 1:
            return 0
        return prob[0]


class DecisionTreeFactorLearner(SklearnFactorLearner):
    def __init__(self):
        clf = tree.DecisionTreeClassifier()
        super(DecisionTreeFactorLearner, self).__init__(clf)


FACTOR_LEARNERS['decision_tree'] = DecisionTreeFactorLearner


class SvmFactorLearner(SklearnFactorLearner):
    def __init__(self):
        clf = svm.SVC(gamma='auto', probability=True)
        super(SvmFactorLearner, self).__init__(clf)


FACTOR_LEARNERS['svm'] = SvmFactorLearner


# TODO(Kevin): Figure out how to deal with SklearnFactorLearner that do not have a predict_proba func
# class LeastSquaresFactorLearner(SklearnFactorLearner):
#     def __init__(self):
#         clf = linear_model.LinearRegression()
#         super(LeastSquaresFactorLearner, self).__init__(clf)


# FACTOR_LEARNERS['least_squares'] = LeastSquaresFactorLearner


class ClosedFormFactorLearner():
    def __init__(self):
        self._joint_assignments = {}
        self._predict_assignment = None
        self._num_var_props = None
        self._num_mb_props = None
        self._num_vars = None
        self._must_fit = True
        self._y_mb = []

    def fit(self, assignment):
        """ Fit a model.

        Choose the canonical assignment y as the largest number in the joint_assignments that matches the assignment of
        the objects in assignment. Since the objects only have one state in goal inference we only want to consider
        canonical assignments using thoes values.

        """
        obj_val_getter = operator.itemgetter(*assign_obj_val_indices(assignment))
        assign_vals = assign_to_vals(assignment)
        obj_vals = obj_val_getter(assign_vals)
        valid_joint_assignmnets = {key: value for key, value in self._joint_assignments.iteritems()
                                   if obj_val_getter(key) == obj_vals}
        self._canonical_assignment = max(valid_joint_assignmnets.iteritems(), key=operator.itemgetter(1))[0]
        # Get the total number of examples canonical markov blanket
        if self._num_mb_props != 0:
            mb_getter = operator.itemgetter(*list(range(self._num_var_props, self._num_var_props + self._num_mb_props)))
            self._y_mb = mb_getter(self._canonical_assignment)
            self._num_y_mb_samps = sum([count for assignment, count in self._joint_assignments.iteritems()
                                       if mb_getter(assignment) == self._y_mb])
            self._y_mb = [x for x in self._y_mb]
        else:
            self._y_mb = []
            self._num_y_mb_samps = sum(self._joint_assignments.values())
        # Done fitting
        self._must_fit = False

    def observe(self, obs, markov_blanket):
        # TODO(Kevin): Ensure markov_blanket is in a specific order
        # New observation therfore we must refit
        self._must_fit = True
        # Make a key out of the joint assignment
        var_props = assign_to_vals(obs)
        mb_props = assign_to_vals(markov_blanket)
        key = tuple(var_props + mb_props)
        # Save representative vars, sizes of obs and sizes of markov_blanket
        if self._num_vars is None:
            self._num_vars = len(obs)
            self._num_var_props = len(var_props)
            self._num_mb_props = len(mb_props)
        # Increment the number of times that joint assignment has been seen
        try:
            self._joint_assignments[key] += 1
        except KeyError:
            self._joint_assignments[key] = 1

    def predict(self, assignment):
        if (self._must_fit or
                self._predict_assignment is None or
                not assign_eq(self._predict_assignment, assignment)):
            self.fit(assignment)
            self._predict_assignment = assignment

        # Calculate factor as equ 7 from "Learning Factor Graphs in Polynomial Time and Sample Complexity"
        exponent = 0
        # Loop over powerset of variables in factor
        for subset in srp_md.powerset(range(self._num_vars)):
            # Add if even subtract if odd
            mult = -1 if (self._num_vars - len(subset)) % 2 else 1
            # Use canonical assignment for all vars not in the subset
            sigma_assignment = list(self._canonical_assignment[:self._num_var_props])
            for var_index in subset:
                prop_index = sum([len(assignment.values()[i]) for i in range(var_index)])
                prop_end_index = prop_index + len(assignment.values()[var_index])
                sigma_assignment[prop_index:prop_end_index] = assignment.values()[var_index].values()

            # P(A|MB(A)) = P(A,MB(A)) / P(MB(A))
            # P(A|MB(A)) aprox = (N(A,MB(A)) / num_samples) / (N(MB(A)) / num_samples)
            #            aprox = N(A,MB(A)) / N(MB(A))
            try:
                count_sigma_mb = float(self._joint_assignments[tuple(sigma_assignment + self._y_mb)])
                log_prob_sigma_given_mb = log(count_sigma_mb / self._num_y_mb_samps)
            except (ValueError, KeyError) as e:
                # Log of 0 clip to large negative number
                log_prob_sigma_given_mb = log(0.001 / self._num_y_mb_samps)
            # Accumulate
            exponent += mult * log_prob_sigma_given_mb

        return exp(exponent)


FACTOR_LEARNERS['closed_form'] = ClosedFormFactorLearner
