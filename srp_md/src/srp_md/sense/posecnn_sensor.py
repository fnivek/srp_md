from __future__ import absolute_import
from builtins import range
from . import sense
import random


class PoseCNNSensor(sense.BaseSensor):
    def __init__(self):
        super(PoseCNNSensor, self).__init__()
        self._objs = []
        self._poses = []
        self._num_objs = 3
        objects = ["A", "B", "C", "D", "E", "F", "G"]
        self._objs = [random.choice(objects) for _ in range(self._num_objs)]

    def simulate_posecnn(self):
        self._poses = [[random.random() for _ in range(7)] for _ in range(self._num_objs)]
        return (self._objs, self._poses)

    def process_data(self, data):
        processed_data = self.simulate_posecnn()
        return processed_data


sense.sensors['posecnn_sensor'] = PoseCNNSensor
