from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

# List of all goal generators
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. goal_generators['base_goal_generator'] = BaseGoalGenerator
goal_generators = {}


class BaseGoalGenerator(with_metaclass(ABCMeta, object)):
    def __init__(self):
        pass

    @abstractmethod
    def generate_goal(self, factors=None):
        """ Generate a goal.

        TODO(Kevin)

        """
        pass
