from builtins import range
from random import choice, shuffle, random, randint
import logging
import operator
from functools import reduce
from itertools import chain, combinations
import numpy as np
import time
import argparse

# Utils logger
logger = logging.getLogger(__name__)


def reservoir_sample(items, k):
    """ Select k items from items.

    Inputs:
      items - an iterable to choose k from
      k - number to choose

    Returns:
      The k randomly chosen samples

    Assumptions:
      len(items) >= k

    """
    samples = list(items[:k])
    n = k
    for i in range(k, len(items)):
        n += 1
        index = randint(0, n)
        if index < k:
            samples[index] = items[i]
    shuffle(samples)  # There are better ways to get a random order
    return samples


def ncr(n, r):
    r = min(r, n - r)
    numer = reduce(operator.mul, range(n, n - r, -1), 1)
    denom = reduce(operator.mul, range(1, r + 1), 1)
    return numer / denom


def tri_num(n):
    num = 0
    for i in range(n):
        num += i + 1
    return num


def iterate_recursively(it, allowed_iterables=(list, tuple)):
    if isinstance(it, allowed_iterables):
        for item in it:
            for value in iterate_recursively(item, allowed_iterables):
                yield value
    else:
        yield it


def count_dicts(din, dout=None):
    """Count values in dictionary.

    For the set of keys in d1 and d2 make a dictionary. For each such dictionary add each value from d1[key], d2[key] as
    keys and set the vaule to the number of occurances of the value.

    """
    if dout is None:
        dout = {}

    for key, value in din.iteritems():
        # Make the entry for the key if needed
        if key not in dout:
            dout[key] = {}
        # Add value as key if needed
        if value not in dout[key]:
            dout[key][value] = 1
        else:
            dout[key][value] += 1

    return dout


def powerset(it):
    # Copy to list
    it2 = list(it)
    return chain.from_iterable(combinations(it2, r) for r in range(len(it2) + 1))


class ConfigMixin(object):
    def __init__(self):
        super(ConfigMixin, self).__init__()
        self._allowed_config_keys = []

    def update_config(self, **kwargs):
        """ Set the sensors mode.

        Inputs:
          **kwargs - other keyword arguments for derived classes

        """
        for key, value in kwargs.iteritems():
            if key not in self._allowed_config_keys:
                raise KeyError('{} is not an allowed to be changed'.format(key))
            setattr(self, key, value)


def pose_difference(pose_1, pose_2):
    position_diff = np.sqrt((pose_1.position.x - pose_2.position.x)**2 + (pose_1.position.y - pose_2.position.y)**2 +
                            (pose_1.position.z - pose_2.position.z)**2)
    orientation_diff = 1 - (pose_1.orientation.x * pose_2.orientation.x + pose_1.orientation.y * pose_2.orientation.y +
                            pose_1.orientation.z * pose_2.orientation.z + pose_1.orientation.w * pose_2.orientation.w)
    return np.sqrt(position_diff**2 + orientation_diff**2)


class GlobalTimer(object):
    """!
    @brief      Global timer for timing algorithms.

                Borg style class for timming the algorithm without needing to pass around a timing variable. To use
                Make a new instance of Global timer and run start(<your_name>) and stop(<your_name>). Then results are
                in diff_times[<your_name>].
    """
    _shared_state = {}

    def __init__(self):
        super(GlobalTimer, self).__init__()
        self.__dict__ = self._shared_state
        if not hasattr(self, 'start_times'):
            self.start_times = {}
        if not hasattr(self, 'end_times'):
            self.end_times = {}
        if not hasattr(self, 'diff_times'):
            self.diff_times = {}

    def start(self, name):
        self.start_times[name] = time.time()

    def stop(self, name):
        self.end_times[name] = time.time()
        try:
            self.diff_times[name] = self.end_times[name] - self.start_times[name]
        except KeyError:
            logger.error('No start time for timer named {}'.format(name))
            raise
        return self.diff_times[name]


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
