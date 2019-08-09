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
    r = min(r, n-r)
    numer = reduce(operator.mul, range(n, n-r, -1), 1)
    denom = reduce(operator.mul, range(1, r+1), 1)
    return numer / denom
