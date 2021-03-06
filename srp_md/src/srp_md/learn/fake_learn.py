from __future__ import absolute_import
from . import learn

import logging


class FakeLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def learn(self, obs):
        self._logger.debug('Learn')
        return obs[0]


# Register the learner
learn.learners['fake_learner'] = FakeLearner
