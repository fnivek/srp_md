from srp_md.learn import learn


class Learner(learn.BaseLearner):
    def __init__(self):
        pass

    @classmethod
    def get_name(cls):
        return "fake_learner"

    def learn(self, obs):
        return obs[0]
