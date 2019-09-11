from __future__ import absolute_import
from . import sense

import logging


class ExampleSensor(sense.BaseSensor):
    def __init__(self):
        super(ExampleSensor, self).__init__()
        not_A_and_C = [
            ([True, True, True], False), ([True, True, False], False),
            ([True, False, True], False), ([True, False, False], False),
            ([False, True, True], True), ([False, True, False], False),
            ([False, False, True], True), ([False, False, False], False)]

        A_and_not_B = [
            ([True, True, True], False), ([True, True, False], False),
            ([True, False, True], True), ([True, False, False], True),
            ([False, True, True], False), ([False, True, False], False),
            ([False, False, True], False), ([False, False, False], False)]

        all_true = [
            ([True, True, True], True), ([True, True, False], True),
            ([True, False, True], True), ([True, False, False], True),
            ([False, True, True], True), ([False, True, False], True),
            ([False, False, True], True), ([False, False, False], True)]

        all_false = [
            ([True, True, True], False), ([True, True, False], False),
            ([True, False, True], False), ([True, False, False], False),
            ([False, True, True], False), ([False, True, False], False),
            ([False, False, True], False), ([False, False, False], False)]

        self._logger = logging.getLogger(__name__)
        self._known_examples = A_and_not_B
        self._num_examples = len(self._known_examples)
        self._index = -1

    def process_data(self, data):
        self._index += 1
        if self._index >= self._num_examples:
            self._logger.debug('We have viewd all examples')
            self._index = 0

        return self._known_examples[self._index]


sense.sensors['example_sensor'] = ExampleSensor
