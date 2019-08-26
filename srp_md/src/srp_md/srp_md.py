""" Semantic Robot Programing Multiple Demonstrations.

Defines a class that encapsulates all functionality.

"""
from __future__ import absolute_import
from __future__ import print_function
from builtins import str
from builtins import object
from . import learn
from . import sense
from . import goal
from . import evaluate

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
    def __init__(self, learner=None, sensor=None, goal_generator=None,
                 goal_evaluator='adapt_goal_evaluator'):
        # Logging
        self._logger = log.getLogger(__name__)

        # Vars
        self._obs = []
        self._goal = None
        self._goal_instance = None
        self._raw_data = None
        self._actions = {1: 'Write demos', 2: 'Load demos', 3: 'Undo demo',
                         4: 'Redo demo', 5: 'Clear demos'}
        self._undoed = []
        self._factors = None
        self.demo_types = ["only_goal", "only_not_goal", "random"]
        self.demo_type = None

        # Set the default srp_md strategies
        self.set_learner(learner)
        self.set_sensor(sensor)
        self.set_goal_generator(goal_generator)
        self.set_goal_evaluator(goal_evaluator)

    def get_num_demos(self):
        return len(self._obs)

    """ Learner.

    Functions to interact with the learner.

    """
    def get_learner(self):
        return self._learner_name

    def set_learner(self, learner):
        self._learner_name = learner
        if self._learner_name is None:
            self._learner = None
        else:
            self._learner = learn.learners[learner]()

    def learn(self):
        if self._learner is None:
            self._logger.error('Please select learner!')
        else:
            self._logger.debug('Learning...')
            if len(self._obs) == 0:
                self._logger.warning('No demo to be learned from!')
            elif self.get_learner() == 'factor_graph_learner':
                self._factors = self._learner.learn(self._obs)
                self._logger.debug('Factors learned: %s', self._factors)
            else:
                self._goal = self._learner.learn(self._obs)
                self._logger.debug('Learned: %s', self._goal)

    """ Sensor.

    Functions to interact with the sensor.

    """
    def get_sensor(self):
        return self._sensor_name

    def set_sensor(self, sensor):
        self._sensor_name = sensor
        if self._sensor_name is None:
            self._sensor = None
        else:
            self._sensor = sense.sensors[sensor]()

    def accept_data(self, data):
        """ Accept Raw Data.

        Accept and save new raw data. Processing should be done in process_data
        function so that accept_data can be used in callbacks that should exit
        quickly.

        """
        self._logger.debug('Accept data: ' + str(data))
        self._raw_data = data

    def process_data(self):
        if self._sensor is None:
            self._logger.error('Please select sensor!')
        elif self.demo_type is None:
            self._logger.error('Please select demo type!')
        else:
            self._logger.debug('Processing: ' + str(self._raw_data))
            self._obs.append(self._sensor.process_data(self.demo_type, self._raw_data))

    """ Goal Generator.

    Functions to interact with the goal generator.

    """
    def get_goal_generator(self):
        return self._goal_generator_name

    def set_goal_generator(self, goal_generator):
        self._goal_generator_name = goal_generator
        if self._goal_generator_name is None:
            self._goal_generator = None
        else:
            self._goal_generator = goal.goal_generators[goal_generator]()

    def generate_goal(self):
        if self._goal_generator is None:
            self._logger.error('Please select goal generator!')
        else:
            if self.get_learner() == 'factor_graph_learner':
                self._goal_instance = self._goal_generator.generate_goal(
                    self._factors, self._sensor.process_data(self.demo_type, self._raw_data))
            else:
                self._goal_instance = self._goal_generator.generate_goal()

    """ Goal Evaluator.

    Functions to evaluate generated goal.

    """
    def set_goal_evaluator(self, goal_evaluator):
        self._goal_evaluator_name = goal_evaluator
        if self._goal_evaluator_name is None:
            self._goal_evaluator = None
        else:
            self._goal_evaluator = evaluate.goal_evaluators[goal_evaluator]()

    def evaluate_goal(self):
        if self._goal_evaluator is None:
            self._logger.error('Please select goal evaluator!')
        elif self._sensor is None:
            self._logger.error('Please select sensor!')
        else:
            self._goal_instance = self._sensor.process_data(self.demo_type, self._raw_data)
            self._goal_evaluator.evaluate_goal(self._sensor, self.get_sensor(), self._goal_instance)

    """ Actions.

    Functions to execute actions.

    """
    def write_demos(self, filename="./demos/test.txt"):
        pickle.dump(self._obs, open(filename, 'wb'))
        self._logger.info('Success writing demos to file {}\n'.format(filename))

    def load_demos(self, filename="./demos/test.txt"):
        self._obs = pickle.load(open(filename, 'rb'))
        self._logger.info('Success loading demos from file {}\n'.format(filename))

    def undo_demo(self):
        if len(self._obs) == 0:
            self._logger.info('No demos to undo!\n')
        else:
            last_demo = self._obs.pop()
            self._undoed.append(last_demo)
            self._logger.info('Success undoing last demo\n')

    def redo_demo(self):
        if len(self._undoed) == 0:
            self._logger.info('No undoed demos to redo!\n')
        else:
            last_demo = self._undoed.pop()
            self._obs.append(last_demo)
            self._logger.info('Success redoing last demo\n')

    def clear_demos(self):
        self._obs = []
        self._logger.info('Success clearing demos\n')
