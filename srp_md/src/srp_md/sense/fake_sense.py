from srp_md.sense import sense


class Sensor(sense.BaseSensor):
    def __init__(self):
        pass

    @classmethod
    def get_name(cls):
        return "fake_sensor"

    def process_data(self, data):
        return {'a': 1, 'b': 2, 'c': 3}
