from __future__ import absolute_import
from builtins import range
from . import sense
import random


class FakeSensor(sense.BaseSensor):
    def __init__(self):
        pass

    def process_data(self, demo_type, data):
        return ([random.choice([True, False]) for _ in range(4)],
                random.choice([True, False]))


sense.sensors['fake_sensor'] = FakeSensor
