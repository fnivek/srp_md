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
        self._allowed_config_keys = []

    @abstractmethod
    def learn(self, obs):
        """ Learn.

        From a iterable of world states (obs) return a learned model of the
        goal.

        """
        pass

    def update_config(self, **kwargs):
        """ Set the learners mode.

        Inputs:
          **kwargs - other keyword arguments for derived classes

        """
        for key, value in kwargs.iteritems():
            if key not in self._allowed_config_keys:
                raise KeyError('{} is not an allowed to be changed'.format(key))
            setattr(self, key, value)
