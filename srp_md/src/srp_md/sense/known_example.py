import sense
import random

class ExampleSensor(sense.BaseSensor):
    def __init__(self):
        self.known_examples = [([True, True, True], False), ([True, True, False], False), ([True, False, True], False),
([True, False, False], False), ([False, True, True], True), ([False, True, False], False), ([False, False, True], True),
([False, False, False], False)]

    def process_data(self, data):
        if len(self.known_examples) != 0:
            return self.known_examples.pop(0)
        else:
            print 'We have viewd all examples.'
            return ()


sense.sensors['example_sensor'] = ExampleSensor

not_A_and_C = [([True, True, True], False), ([True, True, False], False), ([True, False, True], False),
([True, False, False], False), ([False, True, True], True), ([False, True, False], False), ([False, False, True], True),
([False, False, False], False)]

A_and_not_B = [([True, True, True], False), ([True, True, False], False), ([True, False, True], True),
([True, False, False], True), ([False, True, True], False), ([False, True, False], False), ([False, False, True], False),
([False, False, False], False)]

all_true = [([True, True, True], True), ([True, True, False], True), ([True, False, True], True),
([True, False, False], True), ([False, True, True], True), ([False, True, False], True), ([False, False, True], True),
([False, False, False], True)]

all_false = [([True, True, True], False), ([True, True, False], False), ([True, False, True], False),
([True, False, False], False), ([False, True, True], False), ([False, True, False], False), ([False, False, True], False),
([False, False, False], False)]

