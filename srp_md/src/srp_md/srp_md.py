""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
from __future__ import absolute_import
from __future__ import print_function
from builtins import str
from builtins import object
from . import learn
from . import sense

# Python imports
import logging as log
import pickle


class SrpMd(object):
    """ Semantic Robot Programing Multiple Demonstrations.

    Implements a semantic robot programing with multiple demonstrations. The
    algorithms can be configured with the set_<x> functions. See the associated
    python package for avaliable algorithms.

    """
    # TODO(Kevin): Set defaults when they exist
    def __init__(self, learner='fake_learner', sensor='fake_sensor'):
        # Logging
        self._logger = log.getLogger(__name__)

        # Vars
        self._obs = []
        self._goal = None
        self._raw_data = None
        self._num_demos = 0
        self._actions = {0: 'write_demos', 1: 'load_demos', 2: 'undo_demo',
                         3: 'redo_demo', 4: 'clear_demos'}
        self._undoed = []

        # Set the default srp_md strategies
        self.set_learner(learner)
        self.set_sensor(sensor)

    def get_num_demos(self):
        return self._num_demos

    """ Learner.

    Functions to interact with the learner.

    """
    def get_learner(self):
        return self._learner_name

    def set_learner(self, learner):
        self._learner_name = learner
        self._learner = learn.learners[learner]()

    def learn(self):
        self._logger.debug('Learning...')
        self._goal = self._learner.learn(self._obs)
        self._logger.debug('Learned: %s', self._goal)

    """ Sensor.

    Functions to interact with the sensor.

    """
    def get_sensor(self):
        return self._sensor_name

    def set_sensor(self, sensor):
        self._sensor_name = sensor
        self._sensor = sense.sensors[sensor]()

    def accept_data(self, data):
        """ Accept Raw Data.

        Accept and save new raw data. Processing should be done in process_data
        function so that accept_data can be used in callbacks that should exit
        quickly.

        """
        self._logger.debug('Accept data: ' + str(data))
        self._raw_data = data
        self._num_demos += 1

    def process_data(self):
        self._logger.debug('Processing: ' + str(self._raw_data))
        self._obs.append(self._sensor.process_data(self._raw_data))

    """ Actions.

    Functions to execute actions.

    """
    def write_demos(self, filename):
        pickle.dump(self._obs, open(filename, 'wb'))
        print('Success in writing demos to file {}'.format(filename))
        # print 'Demos: {}'.format(self._obs)

    def load_demos(self, filename):
        self._obs = pickle.load(open(filename, 'rb'))
        print('Success in loading demos from file {}'.format(filename))
        # print 'Demos: {}'.format(self._obs)

    def undo_demo(self):
        last_demo = self._obs.pop()
        self._undoed.append(last_demo)
        print('Success in undoing last demo')
        # print 'Demos: {}'.format(self._obs)

    def redo_demo(self):
        last_demo = self._undoed.pop()
        self._obs.append(last_demo)
        print('Success in redoing last demo')
        # print 'Demos: {}'.format(self._obs)

    def clear_demos(self):
        self._obs = []
        print('Success in clearing demos')
        # print 'Demos: {}'.format(self._obs)
