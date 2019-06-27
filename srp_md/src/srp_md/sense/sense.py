import abc
from abc import ABCMeta, abstractmethod


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
