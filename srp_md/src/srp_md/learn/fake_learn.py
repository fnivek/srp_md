from srp_md.learn import learn


class Learner(learn.BaseLearner):
    def __init__(self):
        pass

    def learn(self, obs):
        return obs[0]
