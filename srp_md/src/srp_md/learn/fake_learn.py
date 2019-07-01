import learn

import logging


class FakeLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    @classmethod
    def get_name(cls):
        return "fake_learner"

    def learn(self, obs):
        self._logger.debug('Learn')
        return obs[0]


# Register the learner
learn.learners['fake_learner'] = FakeLearner
