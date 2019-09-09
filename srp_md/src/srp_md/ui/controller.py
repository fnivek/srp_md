""" Controller.

Handles interactions between a view and the model.

"""
from builtins import object


class Controller(object):
    def __init__(self, model):
        self._model = model
        self._errors = []

    def pop_errors(self):
        errors = list(self._errors)
        self._errors = []
        return errors

    def set_learner(self, learner):
        self._model.set_learner(learner)

    def update_learner_config(self, **kwargs):
        self._model.update_learner_config(**kwargs)

    def learn(self):
        self._model.learn()

    def set_sensor(self, sensor):
        self._model.set_sensor(sensor)

    def update_sensor_config(self, **kwargs):
        self._model.update_sensor_config(**kwargs)

    def generate_demo(self):
        self._model.accept_data([])
        self._model.process_data()

    def set_goal_generator(self, goal_generator):
        self._model.set_goal_generator(goal_generator)

    def generate_goal(self):
        try:
            self._model.generate_goal()
        except ValueError:
            self._errors.append("Failed to generate goal")

    def evaluate_goal(self):
        self._model.evaluate_goal()

    def accept_data(self):
        self._model.accept_data([])

    def process_data(self):
        try:
            self._model.process_data()
        except ValueError:
            self._errors.append("Failed to process data")

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

    def show_graph(self):
        self._model.show_graph(self._model._current_graph)
