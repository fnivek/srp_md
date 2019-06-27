from srp_md.sense import sense


class Sensor(sense.BaseSensor):
    def __init__(self):
        pass

    def accept_data(self, data):
        pass

    def process_data(self):
        return {'a': 1, 'b': 2, 'c': 3}
