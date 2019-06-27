""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
from learn import fake_learn
from sense import fake_sense


class SrpMd:
    """ Semantic Robot Programing Multiple Demonstrations.

    Implements a semantic robot programing with multiple demonstrations. The
    algorithms can be configured with the set_<x> functions. See the associated
    python package for avaliable algorithms.

    """
    learners = {
        fake_learn.Learner.get_name(): fake_learn.Learner,
    }
    sensors = {
        fake_sense.Sensor.get_name(): fake_sense.Sensor,
    }

    def __init__(self):
        # Set the default srp_md strategies
        # TODO(Kevin): Set these to defaults when they exist
        self._learner = None
        self._sensor = None

        # Vars
        self._obs = []
        self._goal = None
        self._raw_data = None

    """ Learner.

    Functions to interact with the learner.

    """
    def set_learner(self, learner):
        self._learner = self.learners[learner]()

    def learn(self):
        print 'Learning...'
        self._goal = self._learner.learn(self._obs)
        print 'Learned:', self._goal

    """ Sensor.

    Functions to interact with the sensor.

    """
    def set_sensor(self, sensor):
        self._sensor = self.sensors[sensor]()

    def accept_data(self, data):
        """ Accept Raw Data.

        Accept and save new raw data. Processing should be done in process_data
        function so that accept_data can be used in callbacks that should exit
        quickly.

        """
        print 'Accept data:', data
        self._raw_data = data

    def process_data(self):
        print 'Processing:', self._raw_data
        self._obs.append(self._sensor.process_data(self._raw_data))
