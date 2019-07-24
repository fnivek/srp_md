from builtins import object
import abc
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass

# List of all sensors
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. sensors['base_sensor'] = BaseSensor
sensors = {}


class BaseSensor(with_metaclass(ABCMeta, object)):
    def __init__(self):
        pass

    @abstractmethod
    def process_data(self, data):
        """ Process data.

        Processes data and return the world state.

        """
        pass
