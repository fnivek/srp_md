import abc
from abc import ABCMeta, abstractmethod

# List of all sensors
# Each subclass must register itself by adding its name and pointer to class
# in the dictionary i.e. sensors['base_sensor'] = BaseSensor
sensors = {}


class BaseSensor():
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def process_data(self, data):
        """ Process data.

        Processes data and return the world state.

        """
        pass
