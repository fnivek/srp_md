import sense
import random


class FakeSensor(sense.BaseSensor):
    def __init__(self):
        pass

    def process_data(self, data):
        return ([random.choice([True, False]) for _ in xrange(4)],
                random.choice([True, False]))


sense.sensors['fake_sensor'] = FakeSensor
