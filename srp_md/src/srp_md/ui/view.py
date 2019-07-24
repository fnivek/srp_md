""" View.

An abstract base class for views.

"""
from builtins import object
from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass


class BaseView(with_metaclass(ABCMeta, object)):
    def __init__(self, model, ctrl):
        self._model = model
        self._ctrl = ctrl
        self._running = False

    @property
    def running(self):
        return self._running

    @running.setter
    def running(self, value):
        self._running = value

    @abstractmethod
    def update_from_model(self):
        pass

    @abstractmethod
    def run_once(self):
        pass

    @abstractmethod
    def run(self):
        pass

    def set_learner(self, learner):
        """ Sets the learner by string value. """
        self._ctrl.set_learner(learner)

    def learn(self):
        self._ctrl.learn()
