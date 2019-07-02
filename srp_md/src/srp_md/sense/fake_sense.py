import sense
import random


class FakeSensor(sense.BaseSensor):
    def __init__(self):
        pass

    def process_data(self, data):
        out = {}
        out['a'] = random.random()
        out['b'] = random.random()
        out['c'] = random.random()
        return out


sense.sensors['fake_sensor'] = FakeSensor
