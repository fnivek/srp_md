""" Controller.

Handles interactions between a view and the model.

"""


class Controller:
    def __init__(self, model):
        self._model = model

    def set_learner(self, learner):
        self._model.set_learner(learner)

    def learn(self):
        self._model.learn()

    def set_sensor(self, sensor):
        self._model.set_sensor(sensor)

    # TODO(Kevin): I think this doesn't belong here
    def accept_data(self):
        self._model.accept_data([])

    # TODO(Kevin): I think this doesn't belong here
    def process_data(self):
        self._model.process_data()
