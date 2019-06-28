import learn


class FakeLearner(learn.BaseLearner):
    def __init__(self):
        pass

    @classmethod
    def get_name(cls):
        return "fake_learner"

    def learn(self, obs):
        return obs[0]


# Register the learner
learn.learners['fake_learner'] = FakeLearner
