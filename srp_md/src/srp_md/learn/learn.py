import abc
from abc import ABCMeta, abstractmethod

# List of all learners
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. learners['base_learner'] = BaseLearner
learners = {}


class BaseLearner:
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def learn(self, obs):
        """ Learn.

        From a iterable of world states (obs) return a learned model of the
        goal.

        """
        pass
