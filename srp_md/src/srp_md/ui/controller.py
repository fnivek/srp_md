""" Controller.

Handles interactions between a view and the model.

"""
from builtins import object


class Controller(object):
    def __init__(self, model):
        self._model = model

    def set_learner(self, learner):
        self._model.set_learner(learner)

    def learn(self):
        self._model.learn()

    def set_sensor(self, sensor):
        self._model.set_sensor(sensor)

    def generate_demo(self):
        self._model.accept_data([])
        self._model.process_data()

    def set_goal_generator(self, goal_generator):
        self._model.set_goal_generator(goal_generator)

    # TODO(Kevin): I think this doesn't belong here
    def accept_data(self):
        self._model.accept_data([])

    # TODO(Kevin): I think this doesn't belong here
    def process_data(self):
        self._model.process_data()

    # Actions added
    def write_demos(self, filename):
        self._model.write_demos(filename)

    def load_demos(self, filename):
        self._model.load_demos(filename)

    def undo_demo(self):
        self._model.undo_demo()

    def redo_demo(self):
        self._model.redo_demo()

    def clear_demos(self):
        self._model.clear_demos()
