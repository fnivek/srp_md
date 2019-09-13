from __future__ import absolute_import
from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

from srp_md.utils import ConfigMixin

# List of all goal generators
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. goal_generators['base_goal_generator'] = BaseGoalGenerator
goal_generators = {}


class BaseGoalGenerator(with_metaclass(ABCMeta, object, ConfigMixin)):
    def __init__(self):
        super(BaseGoalGenerator, self).__init__()

    @abstractmethod
    def generate_goal(self, factors=None, obs=None):
        """ Generate a goal.

        TODO(Kevin)

        """
        pass
