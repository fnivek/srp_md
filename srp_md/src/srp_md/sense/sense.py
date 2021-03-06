from __future__ import absolute_import
from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

from srp_md.utils import ConfigMixin

# List of all sensors
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. sensors['base_sensor'] = BaseSensor
sensors = {}
goal_types = {}


class BaseSensor(with_metaclass(ABCMeta, object, ConfigMixin)):
    def __init__(self):
        super(BaseSensor, self).__init__()
        self._demo_type = 'only_goal'
        self._goal_type = None
        self._min_num_objs = 3
        self._max_num_objs = 5
        self._allowed_config_keys.extend(['demo_type', 'goal_type', 'min_num_objs', 'max_num_objs'])
        self._allowed_demo_types = ['only_goal', 'only_not_goal', 'random']

    @abstractmethod
    def process_data(self, data):
        """ Process data.

        Processes data and return the world state.

        """
        pass

    @property
    def demo_type(self):
        return self._demo_type

    @demo_type.setter
    def demo_type(self, mode):
        if mode not in self._allowed_demo_types:
            raise KeyError('{} is not an allowed demo type'.format(mode))
        self._demo_type = mode

    @property
    def goal_type(self):
        return self._goal_type

    @goal_type.setter
    def goal_type(self, mode):
        self._goal_type = mode

    @property
    def min_num_objs(self):
        return self._min_num_objs

    @min_num_objs.setter
    def min_num_objs(self, min_num_objs):
        self._min_num_objs = min_num_objs

    @property
    def max_num_objs(self):
        return self._max_num_objs

    @max_num_objs.setter
    def max_num_objs(self, max_num_objs):
        self._max_num_objs = max_num_objs
