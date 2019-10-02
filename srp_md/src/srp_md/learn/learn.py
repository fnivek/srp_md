from __future__ import absolute_import
from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

from srp_md.utils import ConfigMixin

# List of all learners
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. learners['base_learner'] = BaseLearner
learners = {}


class BaseLearner(with_metaclass(ABCMeta, object, ConfigMixin)):
    def __init__(self):
        super(BaseLearner, self).__init__()
        self._allowed_config_keys = []

    @abstractmethod
    def learn(self, obs):
        """ Learn.

        From a iterable of world states (obs) return a learned model of the
        goal.

        """
        pass
