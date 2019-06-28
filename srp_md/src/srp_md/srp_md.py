""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
import learn
import sense


class SrpMd:
    """ Semantic Robot Programing Multiple Demonstrations.

    Implements a semantic robot programing with multiple demonstrations. The
    algorithms can be configured with the set_<x> functions. See the associated
    python package for avaliable algorithms.

    """
    # TODO(Kevin): Set defaults when they exist
    def __init__(self, learner='fake_learner', sensor='fake_sensor'):
        # Vars
        self._obs = []
        self._goal = None
        self._raw_data = None

        # Set the default srp_md strategies
        self.set_learner(learner)
        self.set_sensor(sensor)

    """ Learner.

    Functions to interact with the learner.

    """
    def set_learner(self, learner):
        self._learner = learn.learners[learner]()

    def learn(self):
        print 'Learning...'
        self._goal = self._learner.learn(self._obs)
        print 'Learned:', self._goal

    """ Sensor.

    Functions to interact with the sensor.

    """
    def set_sensor(self, sensor):
        self._sensor = sense.sensors[sensor]()

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
