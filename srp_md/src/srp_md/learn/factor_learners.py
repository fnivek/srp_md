# Python
import string
from random import random

# Scikit
from sklearn import linear_model, tree, preprocessing
from sklearn.pipeline import Pipeline

# Numpy
import numpy

# Project
import srp_md


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

    def observe(self, obs):
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

    def observe(self, obs):
        self._data.append(obs)
        self._target.append(1)
        self._must_fit = True

    def predict(self, assignment):
        # Fit data if needed
        if self._must_fit:
            print('self._data: {}'.format(self._data))
            print('self._target: {}'.format(self._target))
            self._must_fit = False
            self._pipe.fit(self._data, self._target)
            # print('self._pipe[\'enc\'].coef_: {}'.format(self._pipe['enc'].coef_))
        # Predict probability
        # TODO(Kevin): I don't think these should be sorted anymore
        value_tuple = tuple(sorted(value for value in assignment.values()))
        print('value_tuple: {}'.format(value_tuple))
        probs = self._pipe.predict([value_tuple])
        print('probs: {}'.format(probs))

        # relation = [value for value in assignment.values() if value in srp_md.SceneGraph.RELATION_STRS][0]
        # if relation in self._clf.classes_:
        #     probs_index = numpy.where(self._clf.classes_ == relation)
        #     return probs[0][probs_index[0][0]]

        return 0
