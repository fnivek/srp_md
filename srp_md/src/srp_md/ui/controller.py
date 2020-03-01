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

    def get_demo(self):
        self._model.accept_data("demonstration")

    def get_init_scene(self):
        self._model.accept_data("initial scene")

    def process_data(self):
        try:
            self._model.process_data()
        except ValueError as e:
            self._errors.append("Failed to process data: {}".format(e))

    def set_goal_generator(self, goal_generator):
        self._model.set_goal_generator(goal_generator)

    def generate_goal(self):
        try:
            self._model.generate_goal()
        except ValueError as e:
            self._errors.append("Failed to generate goal: {}".format(e))

    def evaluate_goal(self):
        self._model.evaluate_goal()

    def plan(self):
        self._model.plan()

    def write_demos(self, filename):
        self._model.write_demos(filename)

    def load_demos(self, filename):
        self._model.load_demos(filename)

    def write_inits(self, filename):
        self._model.write_inits(filename)

    def load_inits(self, filename):
        self._model.load_inits(filename)

    def write_goals(self, filename):
        self._model.write_goals(filename)

    def undo_demo(self):
        self._model.undo_demo()

    def redo_demo(self):
        self._model.redo_demo()

    def clear_demos(self):
        self._model.clear_demos()

    def clear_inits(self):
        self._model.clear_inits()

    def clear_goals(self):
        self._model.clear_goals()

    def show_graph(self):
        self._model.show_graph()

    def act(self):
        self._model.act()

    def set_keyframe(self, demo_num, keyframe_num):
        self._model.accept_keyframe(demo_num, keyframe_num)

    def undo_keyframe(self):
        self._model.undo_keyframe()

    def clear_keyframes(self, demo_num):
        self._model.clear_keyframes(demo_num)

    def write_keyframes_demos(self, dirname):
        self._model.write_keyframes_demos(dirname)

    def load_keyframes_demos(self, filename):
        self._model.load_keyframes_demos(filename)

    def append_keyframes_demos(self, filename):
        self._model.append_keyframes_demos(filename)

    def process_keyframes(self):
        self._model.process_keyframes()

    def grocery_experiment(self):
        self._model.grocery_experiment()
