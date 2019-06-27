import abc
from abc import ABCMeta, abstractmethod


class BaseSensor():
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def accept_data(self, data):
        """ Accept Raw Data.

        Accept and save new raw data. Processing should be done in process_data
        function so that accept_data can be used in callbacks that should exit
        quickly.

        """
        pass

    @abstractmethod
    def process_data(self):
        """ Process data.

        Processes data recieved in accept data function. Return the world
        state.

        """
        pass
