# Python imports
from __future__ import absolute_import
import logging

# SRP MD imports
from . import learn

# Other imports
import pomegranate


class FactorGraphLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def learn(self, obs):
        self._logger.debug('Learn')
        return None


# Register the learner
learn.learners['factor_graph_learner'] = FactorGraphLearner
