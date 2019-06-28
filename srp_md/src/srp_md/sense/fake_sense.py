import sense


class FakeSensor(sense.BaseSensor):
    def __init__(self):
        pass

    def process_data(self, data):
        return {'a': 1, 'b': 2, 'c': 3}


sense.sensors['fake_sensor'] = FakeSensor
