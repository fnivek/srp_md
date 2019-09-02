from builtins import range
from random import choice, shuffle, random, randint
import logging
import operator
from functools import reduce

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
