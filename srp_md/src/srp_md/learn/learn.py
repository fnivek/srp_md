import abc
from abc import ABCMeta, abstractmethod


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
