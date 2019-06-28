""" View.

An abstract base class for views.

"""
from abc import ABCMeta, abstractmethod


class BaseView:
    __metaclass__ = ABCMeta

    def __init__(self, model, ctrl):
        self._model = model
        self._ctrl = ctrl

    @abstractmethod
    def update_from_model(self):
        pass

    @abstractmethod
    def run(self):
        pass

    def set_learner(self, learner):
        """ Sets the learner by string value. """
        self._ctrl.set_learner(learner)

    def learn(self):
        self._ctrl.learn()
