from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

# List of all learners
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. learners['base_learner'] = BaseLearner
learners = {}


class BaseLearner(with_metaclass(ABCMeta, object)):
    def __init__(self):
        pass

    @abstractmethod
    def learn(self, obs):
        """ Learn.

        From a iterable of world states (obs) return a learned model of the
        goal.

        """
        pass
