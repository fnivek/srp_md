from __future__ import absolute_import
from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

from srp_md.utils import ConfigMixin

# List of all goal generators
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. goal_generators['base_goal_generator'] = BaseGoalGenerator
goal_evaluators = {}


class BaseGoalEvaluator(with_metaclass(ABCMeta, object, ConfigMixin)):
    def __init__(self):
        super(BaseGoalEvaluator, self).__init__()

    @abstractmethod
    def evaluate_goal(self, goal_instance=None):
        """ Evaluate a goal scene.

        """
        pass
